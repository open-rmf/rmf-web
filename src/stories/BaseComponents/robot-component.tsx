import React from 'react';

import Robot, { RobotProps } from '../../components/schedule-visualizer/robot';
import { viewBoxCoords } from './Utils';

interface RobotComponentProps extends RobotProps {
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