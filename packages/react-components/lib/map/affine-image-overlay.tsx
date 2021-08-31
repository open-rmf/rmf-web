import React from 'react';
import { ImageOverlay, ImageOverlayProps } from 'react-leaflet';
import * as RmfModels from 'rmf-models';

export interface AffineImageOverlayProps extends Omit<ImageOverlayProps, 'url'> {
  image: RmfModels.AffineImage;
}

export function AffineImageOverlay({
  image,
  ...otherProps
}: AffineImageOverlayProps): JSX.Element | null {
  return <ImageOverlay url={image.data} {...otherProps} />;
}
