from shapely.geometry import Polygon

class MarginReducer:
    """
    Implementation of Margin Reduction using Shapely.
    Applies safety margin ONLY to exterior boundary, preserving obstacles unchanged.
    """

    @staticmethod
    def shrink(polygon: Polygon, margin_h: float) -> Polygon:
        """
        Contracts a polygon's EXTERIOR ONLY by a distance 'h'.
        Obstacles (holes) are preserved at their original size.
        
        Strategy:
        1. Extract original holes (obstacles are NO-GO zones, no margin needed)
        2. Shrink only the exterior boundary with buffer(-h)
        3. Re-insert original holes into the shrunken polygon
        
        :param polygon: Original shapely Polygon (with holes).
        :param margin_h: Safety distance in meters (h).
        :return: New reduced Polygon with ORIGINAL holes preserved.
        """
        # Save original holes (obstacles)
        original_holes = [list(interior.coords) for interior in polygon.interiors]
        
        # Create polygon from ONLY the exterior (no holes)
        exterior_only = Polygon(polygon.exterior.coords)
        
        # Shrink ONLY the exterior boundary
        # join_style=2 (Mitre) preserves sharp corners
        shrunken_exterior = exterior_only.buffer(-margin_h, join_style=2)
        
        # Handle case where buffer creates MultiPolygon (shouldn't happen but be safe)
        if shrunken_exterior.geom_type == 'MultiPolygon':
            # Keep largest polygon
            shrunken_exterior = max(shrunken_exterior.geoms, key=lambda p: p.area)
        
        # Re-insert ORIGINAL holes (obstacles) into the shrunken exterior
        # Only keep holes that are still inside the shrunken boundary
        valid_holes = []
        for hole in original_holes:
            hole_poly = Polygon(hole)
            # Check if hole is still within shrunken exterior
            if shrunken_exterior.contains(hole_poly) or shrunken_exterior.intersects(hole_poly):
                valid_holes.append(hole)
        
        # Create final polygon with shrunken exterior + original holes
        result = Polygon(shell=shrunken_exterior.exterior.coords, holes=valid_holes)
        
        return result