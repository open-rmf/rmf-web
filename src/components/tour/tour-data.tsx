import { ReactourStep } from 'reactour';

export const tourText: { [step: string]: string } = {
  step1Welcome: 'Welcome to RoMi dashboard',
  step2Zoom:
    'Click on the zoom buttons to change the view of the floor plan. Alternatively, the scroll button on your mouse would work too!',
  step3Floorplan:
    'Use the floor plan button to switch between available levels and enabling / disabling the view of different components.',
  step4Leaflet:
    'Clicking individual components like doors, robots, lifts on the map will open up its corresponding information tab in the Omnipanel.',
  step5Omnipanel:
    'The Omnipanel button on the Dashboard shows the various panel options available.',
  step6MainMenu:
    'Each Panel is clickable and contains a list of the available items with their corresponding states.',
  step7DoorPanel: 'Let us take a look into the 🚪 Doors Panel.',
  step8DoorTab: 'Here is an example of what you will see when a door tab is expanded',
  step9CommandsPanel:
    'The Commands Panel allows you to send different types of requests that will be handled by RoMi.',
  step10LoopRequest:
    'An example is the Loop Request, where RoMi will assign the most suitable robot to perform the task at the point of request.',
  step11Settings: 'The Settings button opens up the drawer for different dashboard settings.',
  step12TrajAnims: 'Trajectory configurations can be changed using the options available',
  step13HelpButton: 'The Help button opens up another drawer with useful tools and resources',
  step14HelpDrawer: `Familiarise yourself with the hotkey options by clicking on the Hotkeys tab. Finally, to open this guide again, click on the Tutorial tab. You're ready to use RoMi, look out for new features ahead!`,
};

export type stepStyling = {
  backgroundColor: string;
  color: string;
  borderRadius: string;
};

export type stepDetails = {
  [primaryKey: string]: ReactourStep;
};
