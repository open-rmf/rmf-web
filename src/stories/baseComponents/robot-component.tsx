import React from 'react';
import { actions } from '@storybook/addon-actions';

import Robot, { RobotProps } from '../../components/schedule-visualizer/robot';
import { viewBoxCoords, componentDisplayStyle } from './utils';

interface RobotComponentProps extends RobotProps {
  renderInfoPanel(): JSX.Element;
}

export default function RobotComponent(props: RobotComponentProps): React.ReactElement {
  const { robot, footprint, colorManager, renderInfoPanel } = props;

  return (
    <div style={componentDisplayStyle.display}>
      {renderInfoPanel()}

      <svg viewBox={viewBoxCoords}>
        <Robot
          robot={robot}
          footprint={footprint}
          colorManager={colorManager}
          {...actions('onClick')}
        />
      </svg>
    </div>
  );
}
