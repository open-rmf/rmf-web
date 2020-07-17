import React from 'react';

import Lift from '../components/schedule-visualizer/lift';

export default { 
    title: 'Lift',
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
    levels: ['L1', 'L2', 'L3'],
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
const currentFloor = 'L1';
const liftStatesProps = {
    available_floors: ["L1", "L2", "L3"],
    current_floor: "L1",
    current_mode: 2,
    destination_floor: "L1",
    door_state: 2,
    lift_name: "Lift1",
    lift_time: {sec: 0, nanosec: 0},
    motion_state: 0,
    session_id: "",
}

export const State_AGV = () => (
    <div>
        <svg viewBox={'0 0 25.794363144785166 14.53525484725833'}>
            <Lift
                lift={lift}
                currentFloor={currentFloor}
                liftState={liftStatesProps}
            />
        </svg>
    </div>
);

export const State_FIRE = () => {
    <div>
        <svg viewBox={'0 0 25.794363144785166 14.53525484725833'}>

        </svg>
    </div>
}