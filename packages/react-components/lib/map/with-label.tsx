import React from 'react';
import ReactDOM from 'react-dom';
import { NameLabelProps, ScaledNameLabel } from './label-marker';
import { LabelsPortalContext } from './labels-overlay';

export type WithLabelProps<P> = P & {
  labelText: NameLabelProps['text'];
  labelSourceX: NameLabelProps['sourceX'];
  labelSourceY: NameLabelProps['sourceY'];
  labelSourceRadius: NameLabelProps['sourceRadius'];
  labelArrowLength?: NameLabelProps['arrowLength'];
  hideLabel?: boolean;
};

/**
 * Wraps a component with a `ScaledNameLabel` that is rendered with a portal on
 * the element in `LabelsPortalContext`.
 */
export const withLabel =
  <P,>(Component: React.ComponentType<P>): React.ComponentType<WithLabelProps<P>> =>
  ({
    labelText,
    labelSourceX,
    labelSourceY,
    labelSourceRadius,
    labelArrowLength,
    hideLabel = false,
    ...componentProps
  }) => {
    const labelsPortal = React.useContext(LabelsPortalContext);
    return (
      <g>
        <>
          <Component {...(componentProps as any)} />
          {!hideLabel &&
            labelsPortal &&
            ReactDOM.createPortal(
              <ScaledNameLabel
                text={labelText}
                sourceX={labelSourceX}
                sourceY={labelSourceY}
                sourceRadius={labelSourceRadius}
                arrowLength={labelArrowLength}
              />,
              labelsPortal,
            )}
        </>
      </g>
    );
  };
