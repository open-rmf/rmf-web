import type { AffineImage } from 'api-client';
import React from 'react';
import { ImageOverlay, ImageOverlayProps } from 'react-leaflet';

export interface AffineImageOverlayProps extends Omit<ImageOverlayProps, 'url'> {
  image: AffineImage;
}

export function AffineImageOverlay({
  image,
  ...otherProps
}: AffineImageOverlayProps): JSX.Element | null {
  return <ImageOverlay url={image.data} {...otherProps} />;
}
