import React from 'react';

interface ImageIconProps {
  iconPath: string;
  height: string;
  width: string;
  footprint: string;
  dispatchIconError: React.Dispatch<
    React.SetStateAction<{
      path: string | null;
      error: boolean;
    }>
  >;
}

const ImageIcon = React.forwardRef(function(props: ImageIconProps): React.ReactElement {
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
              console.error(
                'An error occurred while loading the image. Using the default image.',
                error,
              );
              return dispatchIconError(previousVal => {
                return { ...previousVal, error: true };
              });
            }}
          />
        </g>
      )}
    </>
  );
});

export default ImageIcon;
