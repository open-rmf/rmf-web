import React from 'react';
import { Box, Button, createMuiTheme, Typography, IconButton, makeStyles } from '@material-ui/core';
import {
  NavigateNext as NavigateNextIcon,
  NavigateBefore as NavigateBeforeIcon,
} from '@material-ui/icons';
import { OmniPanelViewIndex } from './app';
import { SpotlightValue } from './spotlight-value';
import { ReactourStep } from 'reactour';

type stepStyle = {
  backgroundColor: string;
  color: string;
  borderRadius: string;
};

type tourContent = {
  [primaryKey: string]: ReactourStep;
};

interface createTourProps {
  setTourState: React.Dispatch<React.SetStateAction<boolean>>;
  setTourShowOmniPanel: (view: OmniPanelViewIndex) => void;
  setTourSettingsAndOmniPanel: (
    isSettingsVisible: boolean,
    isOmniPanelVisible: boolean,
    clearSpotlight?: boolean | undefined,
  ) => void;
  OmniPanelViewIndex: any;
  doorSpotlight: SpotlightValue<string> | undefined;
  setDoorSpotlight: React.Dispatch<React.SetStateAction<SpotlightValue<string> | undefined>>;
}

export const createTourSteps = (props: createTourProps) => {
  const {
    setTourState,
    setTourSettingsAndOmniPanel,
    setTourShowOmniPanel,
    OmniPanelViewIndex,
    doorSpotlight,
    setDoorSpotlight,
  } = props;

  const theme = createMuiTheme();
  const stepStyle: stepStyle = {
    backgroundColor: theme.palette.primary.light,
    color: theme.palette.info.contrastText,
    borderRadius: '5px',
  };

  function iconStyles() {
    return {
      navigation: {
        color: theme.palette.info.contrastText,
      },
    };
  }

  const classes = makeStyles(iconStyles)();

  const NavButtons = (
    goTo: Function,
    step: number,
    actionBefore?: () => void,
    lastStep?: boolean,
  ): React.ReactNode => (
    <Box>
      {step > 1 && (
        <IconButton
          onClick={() => {
            goTo(step - 2);
          }}
        >
          <NavigateBeforeIcon className={classes.navigation} />
        </IconButton>
      )}
      {!lastStep && (
        <IconButton
          onClick={() => {
            if (actionBefore) {
              actionBefore();
            }
            goTo(step);
          }}
        >
          <NavigateNextIcon className={classes.navigation} />
        </IconButton>
      )}
      {lastStep && (
        <Button
          variant="contained"
          color="primary"
          onClick={() => {
            if (actionBefore) {
              actionBefore();
            }
            setTourState(false);
          }}
        >
          Start Using Romi
        </Button>
      )}
    </Box>
  );

  const tourContent: tourContent = {
    dashboard: {
      selector: '',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">Welcome to the RoMi dashboard</Typography>
          {NavButtons(goTo, step)}
        </Box>
      ),
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    zoomButtons: {
      selector: '[class="leaflet-control-zoom leaflet-bar leaflet-control"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            Click on the zoom buttons to change the view of the floor plan.Alternatively, the scroll
            button on your mouse would work too!
          </Typography>
          {NavButtons(goTo, step)}
        </Box>
      ),
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    floorPlan: {
      selector: '[class= "leaflet-control-layers leaflet-control"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            Use the floor plan button to switch between available levels and enabling / disabling
            the view of different components.
          </Typography>
          {NavButtons(goTo, step)}
        </Box>
      ),
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    leaflet: {
      selector: '[class="leaflet-image-layer leaflet-zoom-animated"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            Clicking individual components like doors, robots, lifts on the map will open up its
            corresponding information tab in the omnipanel.
          </Typography>
          {NavButtons(goTo, step)}
        </Box>
      ),
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    omnipanelButton: {
      selector: '[data-name="omnipanel-button"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            The Omnipanel Button shows the different panel options available in the dashboard.
            Clicking each item would list different information about it!
          </Typography>
          {NavButtons(goTo, step, () => setTourShowOmniPanel(OmniPanelViewIndex.MainMenu))}
        </Box>
      ),
    },
    mainMenu: {
      selector: '[data-component="MainMenu"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            Each Panel contains a list of the available items and their corresponding states.
          </Typography>
          {NavButtons(goTo, step)}
        </Box>
      ),
      action: () => setTourShowOmniPanel(OmniPanelViewIndex.MainMenu),
    },
    doorsPanel: {
      selector: '[data-item="Doors"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            Let us take a look into the
            <span role="img" aria-label="door emoji">
              🚪
            </span>
            Doors Panel.
          </Typography>
          {NavButtons(goTo, step, () => setTourShowOmniPanel(OmniPanelViewIndex.Doors))}
        </Box>
      ),
      action: () => setTourShowOmniPanel(OmniPanelViewIndex.MainMenu),
    },
    doorTab: {
      selector: '[data-name="main_door"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            Here is an example of what you will see when a door tab is expanded
          </Typography>
          {NavButtons(goTo, step, () => setTourShowOmniPanel(OmniPanelViewIndex.MainMenu))}
        </Box>
      ),
      action: () => {
        if (!doorSpotlight) {
          setDoorSpotlight({ value: 'main_door' });
        }
      },
    },
    commandsPanel: {
      selector: '[data-item= "Commands"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            The Commands Panel allows you to send different types of requests that will be handled
            by RoMi.
          </Typography>
          {NavButtons(goTo, step, () => setTourShowOmniPanel(OmniPanelViewIndex.Commands))}
        </Box>
      ),
    },
    loopRequest: {
      selector: '[data-component="LoopForm"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            An example is the Loop Request which can be iterated multiple times. RoMi will assign
            the most suitable robot to perform the task at the point of request.
          </Typography>
          {NavButtons(goTo, step, () => setTourSettingsAndOmniPanel(false, false))}
        </Box>
      ),
      action: () => setTourShowOmniPanel(OmniPanelViewIndex.Commands),
    },
    settingsButton: {
      selector: '[data-name= "settings-button"]',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            The Settings Button opens up the drawer for different dashboard settings.
          </Typography>
          {NavButtons(goTo, step, () => setTourSettingsAndOmniPanel(true, false, true))}
        </Box>
      ),
    },
    trajAnim: {
      selector: '.MuiDrawer-paper',
      content: ({ goTo, step }) => (
        <Box>
          <Typography variant="h6">
            Finally, Trajectory Animations can be changed using the options available. Look out for
            new features ahead!
          </Typography>
          {NavButtons(
            goTo,
            step,
            () => {
              setTourSettingsAndOmniPanel(false, true, true);
              setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
            },
            true,
          )}
        </Box>
      ),
    },
  };

  const tourSteps: ReactourStep[] = [];

  for (var key in tourContent) {
    tourSteps.push({
      selector: tourContent[key].selector,
      content: tourContent[key].content,
      action: tourContent[key].action,
      style: stepStyle,
    });
  }

  return { tourSteps, theme };
};
