import React from 'react';
import { Divider, Typography } from '@material-ui/core';

import LiftComponent from './lift-component';

export default { 
    title: 'Lift',
};

const styles = {
    modeInfoPanel: {
        padding: '2rem'
    },
    modeInfoItem: {
        display: 'flex',
        justifyContent: 'space-between',
        padding: '0.5rem'
    }
};

const lift = {
    depth: 2.5,
    doors: [
        {
            door_type: 1, 
            motion_direction: 1, 
            motion_range: 0, 
            name: 'lift1_front_door',
            v1_x: 8.8,
            v1_y: -2.3,
            v2_x: 7.7,
            v2_y: -4.5
        },
    ],
    levels: ['L1', 'L2', 'L3', 'L4'],
    name: 'Lift1',
    ref_x: 7.1,
    ref_y: -2.8,
    ref_yaw: 0.5,
    wall_graph: {
        edges: [],
        name: "wallgraph",
        params: [],
        vertices: [],
    },
    width: 2.5
};

const liftStates = {
    available_floors: ["L1", "L2", "L3", "L4"],
    available_modes: new Uint8Array(0),
    current_floor: "L1",
    current_mode: 0,
    destination_floor: "L1",
    door_state: 0,
    lift_name: "Lift1",
    lift_time: {sec: 0, nanosec: 0},
    motion_state: 0,
    session_id: "",
}

const AGVState = {
    ... liftStates,
    current_mode: 2,
    door_state: 2,
};

const FIREState = {
    ... liftStates,
    current_mode: 3,
    current_floor: 'L2',
    destination_floor: 'L4',
    motion_state: 1
};

const UNKOWNState = {
    ... liftStates,
    door_state: 1,
    motion_state: 3
}

const HUMANState = {
    ... liftStates,
    current_mode: 1,
    current_floor: 'L3',
    destination_floor: 'L1',
    motion_state: 2
}

const OFFLINEState = {
    ... liftStates,
    current_mode: 4,
}

const EMERGENCYState = {
    ... liftStates,
    current_mode: 5
}

const renderInfoPanel = (mode: string, doorState: string, motionState: string) => {
    return (
        <div style={styles.modeInfoPanel}>
            <Typography align="center" variant="h5">Configurations</Typography>

            <div style={styles.modeInfoItem}>
                <Typography variant="body1">Mode:</Typography>
                <Typography variant="body1">{mode}</Typography>
            </div>

            <Divider />

            <div style={styles.modeInfoItem}>
                <Typography variant="body1">Door State:</Typography>
                <Typography variant="body1">{doorState}</Typography>
            </div>

            <Divider />

            <div style={styles.modeInfoItem}>
                <Typography variant="body1">Motion State:</Typography>
                <Typography variant="body1">{motionState}</Typography>
            </div>
        </div>
    );
}

export const State_AGV = () => (
        <LiftComponent
            renderInfoPanel={() => renderInfoPanel('AGV', 'Open', 'Stopped')}
            lift={lift}
            currentFloor={AGVState.current_floor}
            liftState={AGVState}
        />
);

export const State_FIRE = () => (
    <LiftComponent
        renderInfoPanel={() => renderInfoPanel('FIRE', 'Closed', 'Up')}
        lift={lift}
        currentFloor={FIREState.current_floor}
        liftState={FIREState}
    />
)

export const State_UNKNOWN = () => (
    <LiftComponent
        renderInfoPanel={() => renderInfoPanel('UNKNOWN', 'Moving', 'Unknown')}
        lift={lift}
        currentFloor={UNKOWNState.current_floor}
        liftState={UNKOWNState}
    />
)

export const State_HUMAN = () => (
    <LiftComponent
        renderInfoPanel={() => renderInfoPanel('HUMAN', 'Closed', 'Down')}
        lift={lift}
        currentFloor={HUMANState.current_floor}
        liftState={HUMANState}
    />
)

export const State_OFFLINE = () => (
    <LiftComponent
        renderInfoPanel={() => renderInfoPanel('OFFLINE', 'Closed', 'Stopped')}
        lift={lift}
        currentFloor={OFFLINEState.current_floor}
        liftState={OFFLINEState}
    />
)

export const State_EMERGENCY = () => (
    <LiftComponent
        renderInfoPanel={() => renderInfoPanel('EMERGENCY', 'Closed', 'Stopped')}
        lift={lift}
        currentFloor={EMERGENCYState.current_floor}
        liftState={EMERGENCYState}
    />
)
