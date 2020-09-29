import { SVGOverlay as SVGOverlay_, SVGOverlayProps as SVGOverlayProps_ } from 'react-leaflet';

/**
 * SVGOverlay typings are broken, its missing the required attribute 'bounds'.
 */
export interface SVGOverlayProps extends SVGOverlayProps_ {
  bounds: L.LatLngBoundsExpression;
}

export const SVGOverlay: React.FunctionComponent<SVGOverlayProps> = SVGOverlay_ as any;

export default SVGOverlay;
