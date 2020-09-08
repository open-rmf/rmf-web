import React from 'react';

interface ImageIconProps {
  iconPath: string;
  height: number;
  width: number;
  footprint: number;
  dispatchIconError: React.Dispatch<
    React.SetStateAction<{
      path: string | null;
      error: boolean;
    }>
  >;
}

const ImageIcon = React.memo(
  (props: ImageIconProps): React.ReactElement => {
    const { iconPath, dispatchIconError, height, width, footprint } = props;
    return (
      <>
        {!!iconPath && (
          <g transform={`translate(${-footprint} ${-footprint})`}>
            <image
              href={iconPath}
              height={height}
              width={width}
              onError={error => {
                console.error('An error occurred while loading the image.', error);
                return dispatchIconError(previousVal => {
                  return { ...previousVal, error: true };
                });
              }}
            />
          </g>
        )}
      </>
    );
  },
);

export default ImageIcon;
