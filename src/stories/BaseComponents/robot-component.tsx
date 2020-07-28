import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

import Robot from '../../components/schedule-visualizer/robot';
import ColorManager from '../../components/schedule-visualizer/colors';
import { viewBoxCoords } from './Utils';

export interface RobotComponentProps {
    robot: RomiCore.RobotState;
    footprint: number;
    colorManager: ColorManager;
    renderInfoPanel(): JSX.Element;
}

const styles = {
    display: {
        display: 'grid',
        gridTemplateColumns: '1fr 3fr'
    }
}

export default function RobotComponent(props: RobotComponentProps): React.ReactElement {
    
    const { robot, footprint, colorManager, renderInfoPanel } = props;

    return (
        <div style={styles.display}>
            {renderInfoPanel()}

            <svg viewBox={viewBoxCoords}>  
                <Robot
                    robot={robot}
                    footprint={footprint}
                    colorManager={colorManager}
                />
            </svg>
        </div>
    );
}