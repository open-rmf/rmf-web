import React from 'react';
import { SVGOverlay as SVGOverlay_, SVGOverlayProps as SVGOverlayProps_ } from 'react-leaflet';

/**
 * SVGOverlay typings are broken, its missing the required attribute 'bounds'.
 */
export interface SVGOverlayProps extends SVGOverlayProps_ {
  bounds: L.LatLngBoundsExpression;
}

interface SVGOverlayComponent extends SVGOverlay_<SVGOverlayProps> {
  container?: SVGSVGElement;
}

export const SVGOverlay = (SVGOverlay_ as unknown) as React.ClassType<
  SVGOverlayProps,
  SVGOverlayComponent,
  {
    new (props: SVGOverlayProps, context?: unknown): SVGOverlayComponent;
  }
>;

export default SVGOverlay;
