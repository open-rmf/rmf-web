import React from 'react';
import { Box, Typography, createMuiTheme } from '@material-ui/core';
import { OmniPanelViewIndex } from '../dashboard';
import { SpotlightValue } from '../spotlight-value';
import { ReactourStep } from 'reactour';
import { tourText, stepDetails, stepStyling } from './tour-data';
import { NavButtons } from './tour-navigation-control';
interface createTourProps {
  setTourState: React.Dispatch<React.SetStateAction<boolean>>;
  setShowSettings: React.Dispatch<React.SetStateAction<boolean>>;
  setShowOmniPanel: React.Dispatch<React.SetStateAction<boolean>>;
  setShowHelp: React.Dispatch<React.SetStateAction<boolean>>;
  clearSpotlights: () => void;
  setCurrentView: React.Dispatch<React.SetStateAction<OmniPanelViewIndex>>;
  doorSpotlight: SpotlightValue<string> | undefined;
  setDoorSpotlight: React.Dispatch<React.SetStateAction<SpotlightValue<string> | undefined>>;
}

export const createTourSteps = (props: createTourProps) => {
  const {
    setTourState,
    setShowSettings,
    setShowOmniPanel,
    setShowHelp,
    clearSpotlights,
    setCurrentView,
    doorSpotlight,
    setDoorSpotlight,
  } = props;

  const showSettingsOmniPanelHelpClearSpotlight = (
    isSettingsVisible: boolean,
    isOmniPanelVisible: boolean,
    isHelpVisible: boolean,
    clearSpotlight: boolean,
  ): void => {
    setShowSettings(isSettingsVisible);
    setShowOmniPanel(isOmniPanelVisible);
    setShowHelp(isHelpVisible);
    clearSpotlight && clearSpotlights();
  };

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
      action: () => showSettingsOmniPanelHelpClearSpotlight(false, false, false, true),
    },
    zoomButtons: {
      selector: '[class="leaflet-control-zoom leaflet-bar leaflet-control"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step2Zoom}</Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => showSettingsOmniPanelHelpClearSpotlight(false, false, false, true),
    },
    floorPlan: {
      selector: '[class= "leaflet-control-layers leaflet-control"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step3Floorplan}</Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => showSettingsOmniPanelHelpClearSpotlight(false, false, false, true),
    },
    leaflet: {
      selector: '[class="leaflet-image-layer leaflet-zoom-animated"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step4Leaflet}</Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => showSettingsOmniPanelHelpClearSpotlight(false, false, false, true),
    },
    omnipanelButton: {
      selector: '[id="toggle-omnipanel-btn"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step5Omnipanel}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.MainMenu);
            }}
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
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.Doors);
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
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.MainMenu);
            }}
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.MainMenu);
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
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.Commands);
            }}
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.Doors);
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
              showSettingsOmniPanelHelpClearSpotlight(false, false, false, false);
            }}
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.MainMenu);
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
            handleNextClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(true, false, false, false)
            }
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.Commands);
            }}
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
            handleNextClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(false, false, false, true)
            }
            handleBackClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(false, false, false, true)
            }
          />
        </Box>
      ),
    },
    helpButton: {
      selector: '[id="show-help-btn"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step13HelpButton}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(false, false, true, true)
            }
            handleBackClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(true, false, false, true)
            }
          />
        </Box>
      ),
    },
    helpDrawer: {
      selector: '[id="help-drawer-options"]',
      content: ({ goTo, step }) => (
        <Box id="stepNode">
          <Typography variant="h6">{tourText.step14HelpDrawer}</Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              setCurrentView(OmniPanelViewIndex.MainMenu);
              setTourState(false);
            }}
            handleBackClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(false, false, false, true)
            }
            lastStep={true}
          />
        </Box>
      ),
    },
  };

  const tourSteps: ReactourStep[] = [];

  for (let key in tourContent) {
    tourSteps.push({
      selector: tourContent[key].selector,
      content: tourContent[key].content,
      action: tourContent[key].action,
      style: stepStyle,
    });
  }

  return { tourSteps, theme };
};
