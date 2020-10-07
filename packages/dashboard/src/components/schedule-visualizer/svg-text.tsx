import React from 'react';

export interface SvgTextProps extends React.SVGProps<SVGTextElement> {
  text: string;
  targetWidth?: number;
}

export default function SvgText(props: SvgTextProps): React.ReactElement {
  const { text, targetWidth, ...otherProps } = props;
  const callbackRef = (textElem: SVGTextElement | null) => {
    if (!textElem) {
      return;
    }

    if (!targetWidth) {
      textElem.textContent = text;
      return;
    }

    // fallback to fixed length if `getComputedTextLength()` is not supported
    if (typeof textElem.getComputedTextLength !== 'function') {
      textElem.textContent = text.slice(0, 8);
      return;
    }

    // svg text does not support auto ellipses, this workaround by testing the text length and
    // truncate it bit by bit until it fits the icon. It's a bit hacky but it shouldn't be too bad
    // unless the robot name is mega long.
    for (textElem.textContent = text; textElem.getComputedTextLength() > targetWidth; ) {
      textElem.textContent = textElem.textContent!.slice(0, textElem.textContent!.length - 6) + '…';
    }
  };

  return <text ref={callbackRef} {...otherProps} />;
}
