import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
 
import Door from '../../components/schedule-visualizer/door/door';
 
export interface DoorComponentProps {
   door: RomiCore.Door;
   doorState: RomiCore.DoorState;
   currentMode: number;
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

            <svg viewBox={'0 0 25.794363144785166 14.53525484725833'}>
                <Door
                    door={door}
                    doorState={doorState}
                    currentMode={currentMode}
                />
            </svg>
        </div>
    )
}
