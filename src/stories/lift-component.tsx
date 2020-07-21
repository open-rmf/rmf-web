import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

import Lift from '../components/schedule-visualizer/lift';

export interface LiftComponentProps {
    currentFloor: string;
    lift: RomiCore.Lift;
    liftState: RomiCore.LiftState;
    renderInfoPanel(): JSX.Element;
    currentMode(): JSX.Element;
    motionState(): JSX.Element;
}

const styles = {
    display: {
        display: 'grid',
        gridTemplateColumns: '1fr 1fr 1fr 2fr'
    }
};

export default function LiftComponent(props: LiftComponentProps): JSX.Element {

    const { currentFloor, lift, liftState, renderInfoPanel, currentMode, motionState } = props;

    return (
        <div style={styles.display}>
            { renderInfoPanel() }

            { currentMode() }

            { motionState() }

            <div>
                <svg viewBox={'0 0 13 13'}>
                    <Lift
                        lift={lift}
                        currentFloor={currentFloor}
                        liftState={liftState}
                    />
                </svg>
            </div>
        </div>
    )
}
