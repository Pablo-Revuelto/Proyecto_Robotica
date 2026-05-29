"""Voice utterance → UCI string parsers.

Two implementations:

* `LangChainLLMParser`: uses a LangChain chat model (HuggingFace endpoint by
  default) with a structured prompt that returns a strict JSON `{uci: "..."}`.
  Falls back to a regex when the LLM is unavailable.

* `RegexFallbackParser`: extracts patterns like "e2 a e4", "Cf3", "peon e4",
  Spanish/English mixed. Used for unit-tests and offline mode.

Both follow the `MoveParser` Protocol so `voice_parser_node` depends on the
abstraction (DIP).
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import List, Optional, Protocol


@dataclass(frozen=True)
class ParseAttempt:
    """Output of a parser; UCI string is the lingua franca with the engine."""
    uci: Optional[str]
    confidence: float
    explanation: str


class MoveParser(Protocol):
    def parse(self, utterance: str, fen: str, legal_uci: List[str]) -> ParseAttempt: ...


# ---- Regex fallback ------------------------------------------------------

# No \b word boundaries: ASR often glues a letter to a square (e.g. "alfil de
# d3 a c4" -> "Quil, DD3, AC4."), and boundaries would reject "DD3"/"AC4".
# Any extracted candidate is still validated against legal_uci, so spurious
# file+digit pairs that don't form a legal move are discarded.
_SQUARE_RE = re.compile(r"([a-hA-H])[\s\-]?([1-8])")
_PIECE_WORDS_ES = {
    "peon": "P", "peón": "P", "torre": "R", "caballo": "N", "alfil": "B",
    "dama": "Q", "reina": "Q", "rey": "K",
}
_PIECE_WORDS_EN = {
    "pawn": "P", "rook": "R", "knight": "N", "bishop": "B",
    "queen": "Q", "king": "K",
}


class RegexFallbackParser:
    """Best-effort textual parser that only emits a move when it is legal."""

    def parse(self, utterance: str, fen: str,
              legal_uci: List[str]) -> ParseAttempt:
        text = utterance.lower()

        if "enroque corto" in text or "kingside castle" in text:
            for uci in ("e1g1", "e8g8"):
                if uci in legal_uci:
                    return ParseAttempt(uci, 0.85, "castling short")
        if "enroque largo" in text or "queenside castle" in text:
            for uci in ("e1c1", "e8c8"):
                if uci in legal_uci:
                    return ParseAttempt(uci, 0.85, "castling long")

        squares = [f"{f.lower()}{r}" for f, r in _SQUARE_RE.findall(text)]
        if len(squares) >= 2:
            for promo in ("", "q", "r", "b", "n"):
                cand = squares[0] + squares[1] + promo
                if cand in legal_uci:
                    return ParseAttempt(cand, 0.7, "two-square match")

        if len(squares) == 1:
            target = squares[0]
            matches = [u for u in legal_uci if u.endswith(target)
                       or u.endswith(target + "q")]
            piece_filter = self._piece_filter(text, fen)
            if piece_filter is not None:
                matches = [u for u in matches
                           if self._piece_at(fen, u[:2]) == piece_filter]
            if len(matches) == 1:
                return ParseAttempt(matches[0], 0.6, "single-square + piece hint")

        return ParseAttempt(None, 0.0, "no match")

    @staticmethod
    def _piece_filter(text: str, fen: str) -> Optional[str]:
        for w, sym in _PIECE_WORDS_ES.items():
            if w in text:
                return sym
        for w, sym in _PIECE_WORDS_EN.items():
            if w in text:
                return sym
        return None

    @staticmethod
    def _piece_at(fen: str, square: str) -> Optional[str]:
        import chess
        board = chess.Board(fen)
        piece = board.piece_at(chess.parse_square(square))
        return piece.symbol().upper() if piece else None


# ---- LangChain LLM parser -----------------------------------------------

_SYSTEM_PROMPT = """You convert spoken chess commands (Spanish or English)
into a single UCI move. Reply with a strict JSON object: {"uci": "<uci>"}
or {"uci": null} if the utterance is not a valid chess move.

Context:
- FEN: {fen}
- Legal UCI moves (you MUST pick one of these, or null): {legal_uci}

Examples:
  utterance: "mueve el peón de e2 a e4"   →  {{"uci": "e2e4"}}
  utterance: "caballo a f3"                 →  {{"uci": "g1f3"}}
  utterance: "enroque corto"                →  {{"uci": "e1g1"}}
  utterance: "captura en d5 con la torre"   →  {{"uci": "<the legal rxd5>"}}

Respond ONLY with the JSON object. No prose, no markdown."""


class LangChainLLMParser:
    """LangChain-backed parser. The model is loaded lazily.

    Default backend uses `langchain-huggingface`'s `ChatHuggingFace` with a
    `HuggingFaceEndpoint`, which contacts a HF Inference Endpoint identified by
    `model_id` (requires `HUGGINGFACEHUB_API_TOKEN` in the environment).
    Override `_build_chain` to use a local model or a different provider.
    """

    def __init__(self, model_id: str = "meta-llama/Meta-Llama-3-8B-Instruct",
                 temperature: float = 0.0) -> None:
        self._model_id = model_id
        self._temperature = temperature
        self._fallback = RegexFallbackParser()
        self._chain = None

    def _build_chain(self):
        from langchain_core.prompts import ChatPromptTemplate
        from langchain_core.output_parsers import StrOutputParser
        from langchain_huggingface import ChatHuggingFace, HuggingFaceEndpoint
        llm_endpoint = HuggingFaceEndpoint(
            repo_id=self._model_id,
            temperature=self._temperature,
            max_new_tokens=64,
        )
        llm = ChatHuggingFace(llm=llm_endpoint)
        prompt = ChatPromptTemplate.from_messages([
            ("system", _SYSTEM_PROMPT),
            ("human", "{utterance}"),
        ])
        return prompt | llm | StrOutputParser()

    def parse(self, utterance: str, fen: str,
              legal_uci: List[str]) -> ParseAttempt:
        if self._chain is None:
            try:
                self._chain = self._build_chain()
            except Exception as exc:        # noqa: BLE001
                return self._fallback_with_note(utterance, fen, legal_uci,
                                                f"LLM init failed: {exc}")
        try:
            raw = self._chain.invoke({
                "utterance": utterance,
                "fen": fen,
                "legal_uci": ", ".join(legal_uci),
            })
        except Exception as exc:            # noqa: BLE001
            return self._fallback_with_note(utterance, fen, legal_uci,
                                            f"LLM call failed: {exc}")

        uci = self._extract_uci(raw, legal_uci)
        if uci is None:
            return self._fallback_with_note(utterance, fen, legal_uci,
                                            f"LLM returned no usable UCI: {raw!r}")
        return ParseAttempt(uci, 0.95, "LLM")

    def _fallback_with_note(self, utt: str, fen: str, legal: List[str],
                            note: str) -> ParseAttempt:
        attempt = self._fallback.parse(utt, fen, legal)
        return ParseAttempt(attempt.uci, attempt.confidence * 0.7,
                            f"fallback ({note}): {attempt.explanation}")

    @staticmethod
    def _extract_uci(raw: str, legal_uci: List[str]) -> Optional[str]:
        # Look for a JSON object anywhere in the response.
        match = re.search(r"\{.*?\}", raw, flags=re.DOTALL)
        if match:
            try:
                obj = json.loads(match.group(0))
                cand = obj.get("uci")
                if isinstance(cand, str) and cand in legal_uci:
                    return cand
            except json.JSONDecodeError:
                pass
        # Fallback: any 4–5 char token that is a legal UCI.
        for token in re.findall(r"\b[a-h][1-8][a-h][1-8][qrbn]?\b", raw):
            if token in legal_uci:
                return token
        return None
