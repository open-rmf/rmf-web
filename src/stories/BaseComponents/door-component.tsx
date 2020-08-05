import React from 'react';
import { actions } from '@storybook/addon-actions';
 
import Door, { DoorContainerProps } from '../../components/schedule-visualizer/door/door';
import { viewBoxCoords } from './Utils';
 
interface DoorComponentProps extends DoorContainerProps {
  renderInfoPanel(): JSX.Element;
}

const styles = {
  display: {
    display: 'grid',
    gridTemplateColumns: '1fr 3fr'
  }
}
  

export default function DoorComponent(props: DoorComponentProps): React.ReactElement {

  const { door, doorState, currentMode, renderInfoPanel } = props;

  return (
    <div style={styles.display}>
      { renderInfoPanel() }

      <svg viewBox={viewBoxCoords}>
        <Door
          door={door}
          doorState={doorState}
          currentMode={currentMode}
          {...actions('onClick')}
        />
      </svg>
    </div>
  )
}
