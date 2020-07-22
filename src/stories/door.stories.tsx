import React from 'react';
import { Divider, Typography } from '@material-ui/core';
 
import DoorComponent from './door-component';
 
export default {
   title: 'Door',
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
 
const door = {
   door_type: 0,
   motion_direction: 1,
   motion_range: -1.571,
   name: "main_door",
   v1_x: 10.8,
   v1_y: -2.3,
   v2_x: 7.7,
   v2_y: -5.5
}
 
const doorState = {
   current_mode: {
       value: 0
   },
   door_name: 'main_door',
   door_time: {sec: 0, nanosec: 0}
}
 
const singleSlidingDoor = {
   ... door,
   door_type: 1
}
 
const doubleSldingDoor = {
   ... door,
   door_type: 2
}

const doorTypeMap = [
   'Undefined',
   'Single Sliding Door',
   'Double Sliding Door',
   'Single Telescope Door',
   'Double Telescope Door',
   'Single Swing Door',
   'Double Swing Door'
];

const doorModeMap = [ 'Close', 'Moving', 'Open' ];

const renderInfoPanel = (doorType: string, doorState: string): JSX.Element => {
   return (
       <div style={styles.modeInfoPanel}>
           <Typography align="center" variant="h5">Configurations</Typography>

           <div style={styles.modeInfoItem}>
               <Typography variant="body1">Door Type:</Typography>
               <Typography variant="body1">{doorType}</Typography>
           </div>

           <Divider />

           <div style={styles.modeInfoItem}>
               <Typography variant="body1">Door Mode:</Typography>
               <Typography variant="body1">{doorState}</Typography>
           </div>
       </div>
   );
}

const availableDoorType = (): JSX.Element => {
   return (
      <div style={styles.modeInfoPanel}>
         <div>
            <Typography align="center" variant="h5">Available Door types</Typography>
         </div>

         {
            doorTypeMap.map((key, index) => {
               return  (
                  <React.Fragment>
                     <div style={styles.modeInfoItem} key={key}>
                           <Typography variant="body1">{index}</Typography>
                           <Typography variant="body1">{key}</Typography>
                     </div>
                     <Divider />
                  </React.Fragment>
               );
            })
         }
      </div>
   );
}

const doorMode = (): JSX.Element => {
   return (
      <div style={styles.modeInfoPanel}>
         <div>
            <Typography align="center" variant="h5">Available Door types</Typography>
         </div>

         {
            doorModeMap.map((key, index) => {
               return  (
                  <React.Fragment>
                     <div style={styles.modeInfoItem} key={key}>
                           <Typography variant="body1">{index}</Typography>
                           <Typography variant="body1">{key}</Typography>
                     </div>
                     <Divider />
                  </React.Fragment>
               );
            })
         }
      </div>
   );
}
 
export const SinglePanelDoors = () => (
    <DoorComponent
        door={singleSlidingDoor}
        doorState={doorState}
        currentMode={0}
        renderInfoPanel={() =>  renderInfoPanel('Single Sliding Door', 'Close')}
        availableDoorType={() => availableDoorType()}
        doorMode={() => doorMode()}
    />
);
 
export const DoublePanelDoors = () => (
    <DoorComponent
        door={doubleSldingDoor}
        doorState={doorState}
        currentMode={0}
        renderInfoPanel={() =>  renderInfoPanel('Double Sliding Door', 'Close')}
        availableDoorType={() => availableDoorType()}
        doorMode={() => doorMode()}
    />
)
 
export const MovingDoor = () => (
   <DoorComponent
      door={singleSlidingDoor}
      doorState={doorState}
      currentMode={1}
      renderInfoPanel={() =>  renderInfoPanel('Single Sliding Door', 'Moving')}
      availableDoorType={() => availableDoorType()}
      doorMode={() => doorMode()}
   />
)

export const ClosingDoor = () => (
   <DoorComponent
      door={singleSlidingDoor}
      doorState={doorState}
      currentMode={2}
      renderInfoPanel={() =>  renderInfoPanel('Single Sliding Door', 'Open')}
      availableDoorType={() => availableDoorType()}
      doorMode={() => doorMode()}
   />
)
