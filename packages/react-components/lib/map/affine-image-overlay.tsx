import React from 'react';
import { makeStyles } from '@material-ui/core';
import { ImageOverlay, ImageOverlayProps } from 'react-leaflet';
import * as RmfModels from 'rmf-models';

const imageOverlayStyles = makeStyles((theme) => ({
  mapImg: {
    filter: theme.mapClass,
  },
}));

export interface AffineImageOverlayProps extends Omit<ImageOverlayProps, 'url'> {
  image: RmfModels.AffineImage;
}

export function AffineImageOverlay({
  image,
  ...otherProps
}: AffineImageOverlayProps): JSX.Element | null {
  const classes = imageOverlayStyles();
  return <ImageOverlay className={classes.mapImg} url={image.data} {...otherProps} />;
}
