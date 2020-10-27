import React from 'react';

export interface ImageIconProps extends React.SVGAttributes<SVGImageElement> {
  iconPath: string;
  height: number;
  width: number;
  footprint: number;
}

export const SvgImage = (props: ImageIconProps): JSX.Element => {
  const { iconPath, height, width, footprint, ...otherProps } = props;
  return (
    <>
      {!!iconPath && (
        <g transform={`translate(${-footprint} ${-footprint})`}>
          <image {...otherProps} href={iconPath} height={height} width={width} />
        </g>
      )}
    </>
  );
};

export default SvgImage;
