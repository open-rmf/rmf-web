import React from 'react';
import ReactDOM from 'react-dom';
import { NameLabelProps, ScaledNameLabel } from './label-marker';
import { LabelsPortalContext } from './labels-overlay';

export type WaypointLabelMakerProps = {
  labelText: NameLabelProps['text'];
  labelSourceX: NameLabelProps['sourceX'];
  labelSourceY: NameLabelProps['sourceY'];
  labelSourceRadius: NameLabelProps['sourceRadius'];
  labelArrowLength?: NameLabelProps['arrowLength'];
  hideLabel?: boolean;
};

export const WaypointLabelMaker: React.FC<WaypointLabelMakerProps> = ({
  labelText,
  labelSourceX,
  labelSourceY,
  labelSourceRadius,
  labelArrowLength,
  hideLabel = false,
}) => {
  const labelsPortal = React.useContext(LabelsPortalContext);

  return (
    <g>
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
    </g>
  );
};
