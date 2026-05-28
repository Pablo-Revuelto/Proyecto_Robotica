# Full chess board snapshot.
# `pieces` is row-major, ranks 1..8 then files a..h (64 entries).
# Each entry encodes piece as ChessMove.PIECE_* and color in the top bit
# (0x80 = black, 0 = white, 0x00 piece=PIECE_NONE means empty).
std_msgs/Header header
uint8[64] pieces
string fen
uint8 turn             # ChessMove.COLOR_WHITE / COLOR_BLACK
float32 confidence     # 0..1, perception confidence for the snapshot
