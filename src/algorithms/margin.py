from shapely.geometry import Polygon

class MarginReducer:
    """
    Implementation of Margin Reduction using Shapely.
    Replaces custom math with robust buffer logic to handle obstacles (holes) correctly.
    """

    @staticmethod
    def shrink(polygon: Polygon, margin_h: float) -> Polygon:
        """
        Contracts a polygon by a distance 'h'.
        Uses shapely buffer(-h) which correctly handles holes (obstacles).
        
        :param polygon: Original shapely Polygon (with holes).
        :param margin_h: Safety distance in meters (h).
        :return: New reduced Polygon with expanded holes.
        """
        # buffer with negative distance shrinks the exterior and EXPANDS the holes (safety margin)
        # join_style=2 (Mitre) preserves sharp corners
        # resolution=16 ensures smooth curves if corners are rounded (though mitre avoids most)
        return polygon.buffer(-margin_h, join_style=2)