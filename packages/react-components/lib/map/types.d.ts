export declare module 'react-leaflet' {
  /**
   * SVGOverlay typings are broken, its missing the required attribute 'bounds'.
   */
  interface SVGOverlayProps {
    bounds: L.LatLngBoundsExpression;
  }

  interface SVGOverlay {
    container: SVGSVGElement;
  }
}
