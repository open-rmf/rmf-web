import React from 'react';
import { actions } from '@storybook/addon-actions';

import Lift, { LiftProps } from '../../components/schedule-visualizer/lift';
import { viewBoxCoords, StyleTyping } from './Utils';

interface LiftComponentProps extends LiftProps {
  renderInfoPanel(): JSX.Element;
}

const styles: StyleTyping = {
  display: {
    display: 'grid',
    gridTemplateColumns: '1fr 3fr',
  },
};

export default function LiftComponent(props: LiftComponentProps): React.ReactElement {
  const { currentFloor, lift, liftState, renderInfoPanel } = props;

  return (
    <div style={styles.display}>
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
