from shapely.geometry import Polygon
from shapely.ops import unary_union

class MarginReducer:
    """Inset the field and inflate obstacles to keep h-meter clearance."""

    @staticmethod
    def _largest_polygon(geom):
        if geom.geom_type == 'MultiPolygon':
            return max(geom.geoms, key=lambda p: p.area)
        return geom

    @staticmethod
    def shrink_exterior(polygon: Polygon, margin_h: float) -> Polygon:
        """Inset the outer boundary by `margin_h` meters."""
        exterior_only = Polygon(polygon.exterior.coords)
        shrunken = exterior_only.buffer(-margin_h, join_style=2)

        if shrunken.is_empty:
            return shrunken

        return MarginReducer._largest_polygon(shrunken)

    @staticmethod
    def expand_obstacles(polygon: Polygon, margin_h: float) -> list[Polygon]:
        """Inflate every hole by `margin_h` meters."""
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
        """Merge obstacles separated by less than `margin_h` (no-fit gap).

        Inflates by h/2, unions, then deflates so pairs closer than h
        collapse into a single obstacle.
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
        """Full margin reduction: inset exterior, inflate + merge holes, subtract."""
        shrunken_ext = MarginReducer.shrink_exterior(polygon, margin_h)

        if shrunken_ext.is_empty:
            return shrunken_ext

        expanded_obs = MarginReducer.expand_obstacles(polygon, margin_h)

        if not expanded_obs:
            return shrunken_ext

        merged_obs = MarginReducer._merge_nearby_obstacles(expanded_obs, margin_h)

        result = shrunken_ext
        for obs in merged_obs:
            result = result.difference(obs)

        if result.is_empty:
            return result

        return MarginReducer._largest_polygon(result)

