"""Özel slicing modülü (SAHI benzeri mantık, ama tamamen kendi implementasyonumuz).

Grid tabanlı tile üretimi + overlap. Her tile global (x0, y0) offset'ini taşır;
detector çıktıları bu offset ile tam-kare koordinata projekte edilir.
SAHI kütüphanesi KULLANILMAZ.
"""
from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class Tile:
    x0: int
    y0: int
    x1: int
    y1: int  # global (tam kare) koordinatlar

    @property
    def w(self) -> int:
        return self.x1 - self.x0

    @property
    def h(self) -> int:
        return self.y1 - self.y0


class Tiler:
    """Kareyi grid_cols x grid_rows parçaya, overlap ile böler.

    2x2 = 4 tile, 3x2 = 6 tile. overlap_ratio her tile'ın komşusuyla
    paylaştığı bindirme oranıdır; tile sınırına denk gelen küçük nesnelerin
    en az bir tile'da bütün görünmesini sağlar.
    """

    def __init__(self, grid_cols: int = 2, grid_rows: int = 2,
                 overlap_ratio: float = 0.2):
        assert grid_cols >= 1 and grid_rows >= 1
        assert 0.0 <= overlap_ratio < 0.9
        self.cols = grid_cols
        self.rows = grid_rows
        self.overlap = overlap_ratio
        self._cache_key: Tuple[int, int] = (-1, -1)
        self._tiles: List[Tile] = []

    def compute_tiles(self, img_w: int, img_h: int) -> List[Tile]:
        """Aynı çözünürlük için grid'i cache'ler (her karede yeniden hesap yok)."""
        if self._cache_key == (img_w, img_h):
            return self._tiles

        base_w = img_w / self.cols
        base_h = img_h / self.rows
        ov_x = base_w * self.overlap
        ov_y = base_h * self.overlap

        tiles: List[Tile] = []
        for r in range(self.rows):
            for c in range(self.cols):
                x0 = max(0, int(round(c * base_w - ov_x / 2)))
                y0 = max(0, int(round(r * base_h - ov_y / 2)))
                x1 = min(img_w, int(round((c + 1) * base_w + ov_x / 2)))
                y1 = min(img_h, int(round((r + 1) * base_h + ov_y / 2)))
                tiles.append(Tile(x0, y0, x1, y1))

        self._cache_key = (img_w, img_h)
        self._tiles = tiles
        return tiles
