import React from 'react';
import { actions } from '@storybook/addon-actions';

import Lift, { LiftProps } from '../../components/schedule-visualizer/lift';
import { viewBoxCoords, componentDisplayStyle } from './utils';

interface LiftComponentProps extends LiftProps {
  renderInfoPanel(): JSX.Element;
}

export default function LiftComponent(props: LiftComponentProps): React.ReactElement {
  const { currentFloor, lift, liftState, renderInfoPanel } = props;

  return (
    <div style={componentDisplayStyle.display}>
      {renderInfoPanel()}

      <div>
        <svg viewBox={viewBoxCoords}>
          <Lift
            lift={lift}
            currentFloor={currentFloor}
            liftState={liftState}
            {...actions('onClick')}
          />
        </svg>
      </div>
    </div>
  );
}
