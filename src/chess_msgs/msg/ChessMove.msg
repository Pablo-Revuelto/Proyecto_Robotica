# A chess move expressed in both algebraic notation and world coordinates.

uint8 COLOR_WHITE = 0
uint8 COLOR_BLACK = 1

uint8 PIECE_NONE   = 0
uint8 PIECE_PAWN   = 1
uint8 PIECE_KNIGHT = 2
uint8 PIECE_BISHOP = 3
uint8 PIECE_ROOK   = 4
uint8 PIECE_QUEEN  = 5
uint8 PIECE_KING   = 6

uint8 color
uint8 piece
Square from_square
Square to_square

bool   is_capture
uint8  captured_piece
bool   is_promotion
uint8  promotion_piece
bool   is_castling
bool   is_en_passant

# UCI ("e2e4") and SAN ("Nf3") strings for logging.
string uci
string san
