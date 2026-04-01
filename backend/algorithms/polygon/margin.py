from shapely.geometry import Polygon
from shapely.ops import unary_union

class MarginReducer:
    """
    Shrinks the workable area inward from field edges and expands obstacles outward
    to maintain h-meter clearance on all sides.
    """

    @staticmethod
    def _largest_polygon(geom):
        """Extract the largest polygon from a possibly Multi geometry."""
        if geom.geom_type == 'MultiPolygon':
            return max(geom.geoms, key=lambda p: p.area)
        return geom

    @staticmethod
    def shrink_exterior(polygon: Polygon, margin_h: float) -> Polygon:
        """
        Shrink the field's outer boundary inward by h meters.
        """
        exterior_only = Polygon(polygon.exterior.coords)
        shrunken = exterior_only.buffer(-margin_h, join_style=2)

        if shrunken.is_empty:
            return shrunken

        return MarginReducer._largest_polygon(shrunken)

    @staticmethod
    def expand_obstacles(polygon: Polygon, margin_h: float) -> list[Polygon]:
        """
        Expand each obstacle (hole) outward by h meters.

        :return: List of expanded obstacle polygons.
        """
        expanded = []
        for interior in polygon.interiors:
            hole_poly = Polygon(interior.coords)
            buffered = hole_poly.buffer(margin_h, join_style=2)

            if not buffered.is_empty:
                buffered = MarginReducer._largest_polygon(buffered)
                expanded.append(buffered)

        return expanded

    @staticmethod
    def _merge_nearby_obstacles(
        obstacles: list[Polygon], margin_h: float
    ) -> list[Polygon]:
        """
        Merge obstacles whose gap is < margin_h (drone can't fit between them).
        Uses inflate/union/deflate: buffer by h/2 so gaps < h touch, union, unbuffer.
        """
        if not obstacles:
            return []

        inflated = [obs.buffer(margin_h / 2, join_style=2) for obs in obstacles]
        merged = unary_union(inflated).buffer(-margin_h / 2, join_style=2)

        if merged.is_empty:
            return []
        if merged.geom_type == 'MultiPolygon':
            return [p for p in merged.geoms if not p.is_empty]
        return [merged]

    @staticmethod
    def shrink(polygon: Polygon, margin_h: float) -> Polygon:
        """
        Orchestrates the full margin reduction:
        1. Shrink exterior inward by h
        2. Expand obstacles outward by h
        3. Merge obstacles whose gap < h (drone can't fit between)
        4. Subtract all obstacles from exterior (handles overlaps, holes, and merging with boundary)
        """
        shrunken_ext = MarginReducer.shrink_exterior(polygon, margin_h)

        if shrunken_ext.is_empty:
            return shrunken_ext

        expanded_obs = MarginReducer.expand_obstacles(polygon, margin_h)

        if not expanded_obs:
            return shrunken_ext

        merged_obs = MarginReducer._merge_nearby_obstacles(expanded_obs, margin_h)

        # Subtract all obstacles from exterior.
        # difference() naturally handles all cases:
        # - Fully inside → creates a hole
        # - Overlapping boundary → cuts the exterior
        result = shrunken_ext
        for obs in merged_obs:
            result = result.difference(obs)

        if result.is_empty:
            return result

        return MarginReducer._largest_polygon(result)

