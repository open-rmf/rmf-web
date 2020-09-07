import React from 'react';
import { Typography } from '@material-ui/core';
import { SpotlightValue } from './spotlight-value';

export const createTourSteps = (args: {
  setTourSettingsAndOmniPanel: (
    isSettingsVisible: boolean,
    isOmniPanelVisible: boolean,
    clearSpotlight?: boolean | undefined,
  ) => void;
  setTourShowOmniPanel: (view: typeof OmniPanelViewIndex) => void;
  OmniPanelViewIndex: any;
  doorSpotlight: SpotlightValue<string> | undefined;
  setDoorSpotlight: React.Dispatch<React.SetStateAction<SpotlightValue<string> | undefined>>;
}) => {
  const {
    setTourSettingsAndOmniPanel,
    setTourShowOmniPanel,
    OmniPanelViewIndex,
    doorSpotlight,
    setDoorSpotlight,
  } = args;

  return [
    {
      selector: '',
      content: () => <Typography variant="h6">Welcome to the RoMi dashboard!</Typography>,
      action: () => {
        setTourSettingsAndOmniPanel(false, false, true);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[class="leaflet-control-zoom leaflet-bar leaflet-control"]',
      content: () => (
        <Typography variant="h6">
          Click on the zoom buttons to change the view of the floor plan. Alternatively, the scroll
          button on your mouse would work too!
        </Typography>
      ),
      action: () => {
        setTourSettingsAndOmniPanel(false, false, true);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[class="leaflet-control-layers leaflet-control"]',
      content: () => (
        <Typography variant="h6">
          Use the floor plan button to switch between available levels and enabling / disabling the
          view of different components.
        </Typography>
      ),
      action: () => {
        setTourSettingsAndOmniPanel(false, false, true);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[class="leaflet-image-layer leaflet-zoom-animated"]',
      content: () => (
        <Typography variant="h6">
          Clicking individual components like doors, robots, lifts on the map will open up its
          corresponding information tab in the omnipanel.
        </Typography>
      ),
      action: () => {
        setTourSettingsAndOmniPanel(false, false, true);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[data-name="omnipanel-button"]',
      content: () => (
        <Typography variant="h6">
          The Omnipanel Button shows the different panel options available in the dashboard.
          Clicking each item would list different information about it!
        </Typography>
      ),
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[data-component="MainMenu"]',
      content: () => (
        <Typography variant="h6">
          Each Panel contains a list of the available items and their corresponding states.
        </Typography>
      ),
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '',
      content: () => (
        <Typography variant="h6">
          Let us take a look into the
          <span role="img" aria-label="door emoji">
            🚪
          </span>
          Doors Panel
        </Typography>
      ),
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.Doors);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[data-name="main_door"]',
      content: () => (
        <Typography variant="h6">
          Here is an example of what you will see when a door tab is expanded!
        </Typography>
      ),
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.Doors);
        if (!doorSpotlight) {
          setDoorSpotlight({ value: 'main_door' });
        }
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '',
      content: () => (
        <Typography variant="h6">
          The Commands Panel allows you to send different types of requests that will be handled by
          RoMi.
        </Typography>
      ),
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.Commands);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '[data-component="LoopForm"]',
      content: () => (
        <Typography variant="h6">
          An example is the Loop Request which can be iterated multiple times. RoMi will assign the
          most suitable robot to perform the task at the point of request.
        </Typography>
      ),
      action: () => {
        setTourShowOmniPanel(OmniPanelViewIndex.Commands);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '',
      content: () => (
        <Typography variant="h6">
          The Settings Button opens up the drawer for different dashboard settings
        </Typography>
      ),
      action: () => {
        setTourSettingsAndOmniPanel(true, false);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
    {
      selector: '.MuiDrawer-paper',
      content: () => (
        <Typography variant="h6">
          Finally, Trajectory Animations can be changed using the options available. Look out for
          new features ahead!
        </Typography>
      ),
      action: () => {
        setTourSettingsAndOmniPanel(true, false);
      },
      style: {
        backgroundColor: '#2979ff',
        color: '#fefefe',
        borderRadius: '5px',
      },
    },
  ];
};
