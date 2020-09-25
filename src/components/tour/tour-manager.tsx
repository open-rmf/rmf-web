import React from 'react';
import { Box, Typography, createMuiTheme } from '@material-ui/core';
import { OmniPanelViewIndex } from '../dashboard';
import { SpotlightValue } from '../spotlight-value';
import { ReactourStep } from 'reactour';
import { tourText, stepDetails, stepStyling } from './tour-data';
import { NavButtons } from './tour-navigation-control';
interface createTourProps {
  setTourState: React.Dispatch<React.SetStateAction<boolean>>;
  setTourShowOmniPanel: (view: OmniPanelViewIndex) => void;
  setTourSettingsAndOmniPanel: (
    isSettingsVisible: boolean,
    isOmniPanelVisible: boolean,
    clearSpotlight?: boolean | undefined,
  ) => void;
  OmniPanelViewIndex: typeof OmniPanelViewIndex;
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
  const stepStyle: stepStyling = {
    backgroundColor: theme.palette.primary.light,
    color: theme.palette.info.contrastText,
    borderRadius: '5px',
  };

  const tourContent: stepDetails = {
    dashboard: {
      selector: '',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step1Welcome}</Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    zoomButtons: {
      selector: '[class="leaflet-control-zoom leaflet-bar leaflet-control"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step2Zoom}</Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    floorPlan: {
      selector: '[class= "leaflet-control-layers leaflet-control"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step3Floorplan}</Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    leaflet: {
      selector: '[class="leaflet-image-layer leaflet-zoom-animated"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step4Leaflet}</Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => setTourSettingsAndOmniPanel(false, false, true),
    },
    omnipanelButton: {
      selector: '[id="toggle-omnipanel-btn"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step5Omnipanel}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => setTourShowOmniPanel(OmniPanelViewIndex.MainMenu)}
          />
        </Box>
      ),
    },
    mainMenu: {
      selector: '[data-component="MainMenu"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step6MainMenu}</Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
    },
    doorsPanel: {
      selector: '[data-item="Doors"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step7DoorPanel}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              setTourShowOmniPanel(OmniPanelViewIndex.Doors);
              if (!doorSpotlight) {
                setDoorSpotlight({ value: 'main_door' });
              }
            }}
          />
        </Box>
      ),
    },
    doorTab: {
      selector: '[data-name="main_door"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step8DoorTab}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
            }}
            handleBackClick={() => {
              setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
            }}
          />
        </Box>
      ),
    },
    commandsPanel: {
      selector: '[data-item= "Commands"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step9CommandsPanel}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              setTourShowOmniPanel(OmniPanelViewIndex.Commands);
            }}
            handleBackClick={() => {
              setTourShowOmniPanel(OmniPanelViewIndex.Doors);
            }}
          />
        </Box>
      ),
    },
    loopRequest: {
      selector: '[data-component="LoopForm"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step10LoopRequest}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              setTourSettingsAndOmniPanel(false, false);
            }}
            handleBackClick={() => {
              setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
            }}
          />
        </Box>
      ),
    },
    settingsButton: {
      selector: '[id="show-settings-btn"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step11Settings}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              setTourSettingsAndOmniPanel(true, false, true);
            }}
            handleBackClick={() => setTourShowOmniPanel(OmniPanelViewIndex.Commands)}
          />
        </Box>
      ),
    },
    trajAnim: {
      selector: '.MuiDrawer-paper',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step12TrajAnims}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              setTourSettingsAndOmniPanel(false, true, true);
              setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
              setTourState(false);
            }}
            handleBackClick={() => {
              setTourSettingsAndOmniPanel(false, true, true);
            }}
            lastStep={true}
          />
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
