import { Box, createMuiTheme, Typography } from '@material-ui/core';
import React from 'react';
import { ReactourStep } from 'reactour';
import { OmniPanelViewIndex } from '../dashboard';
import { MainMenuActionType } from '../reducers/main-menu-reducer';
import { stepDetails, stepStyling, tourText } from './tour-data';
import { NavButtons } from './tour-navigation-control';

// eslint-disable-next-line @typescript-eslint/no-unused-vars
async function waitForAnimation(selector: string): Promise<void> {
  // wait for elem to be rendered
  const el = await new Promise<Element>((res) => {
    const timer = setInterval(() => {
      const el = document.querySelector(selector);
      if (!el) {
        return;
      }
      clearInterval(timer);
      res(el);
    }, 10);
  });

  await Promise.all(el.getAnimations().map((anim) => anim.finished));

  // for some reason reactour doesn't capture the whole element immediate after the
  // animation is finished.
  await new Promise((res) => setTimeout(res, 100));
}

interface createTourProps {
  dispatchMenu: any;
  clearSpotlights: () => void;
  doorSpotlight?: () => void;
}

export const createTourSteps = (props: createTourProps) => {
  const { dispatchMenu, clearSpotlights, doorSpotlight } = props;

  const showSettingsOmniPanelHelpClearSpotlight = (
    isSettingsVisible: boolean,
    isOmniPanelVisible: boolean,
    isHelpVisible: boolean,
    clearSpotlight: boolean,
  ): void => {
    dispatchMenu({ type: MainMenuActionType.SHOW_SETTINGS, payload: isSettingsVisible });
    dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: isOmniPanelVisible });
    dispatchMenu({ type: MainMenuActionType.SHOW_HELP, payload: isHelpVisible });
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
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step1">
            {tourText.Welcome.text}
          </Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => showSettingsOmniPanelHelpClearSpotlight(false, false, false, true),
    },
    zoomButtons: {
      selector: '[class="leaflet-control-zoom leaflet-bar leaflet-control"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step2">
            {tourText.Zoom.text}
          </Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => showSettingsOmniPanelHelpClearSpotlight(false, false, false, true),
    },
    floorPlan: {
      selector: '[class= "leaflet-control-layers leaflet-control"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step3">
            {tourText.Floorplan.text}
          </Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => showSettingsOmniPanelHelpClearSpotlight(false, false, false, true),
    },
    leaflet: {
      selector: '[class="leaflet-image-layer leaflet-zoom-animated"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step4">
            {tourText.Leaflet.text}
          </Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
      action: () => showSettingsOmniPanelHelpClearSpotlight(false, false, false, true),
    },
    omnipanelButton: {
      selector: '[id="toggle-omnipanel-btn"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step5">
            {tourText.Omnipanel.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.MainMenu,
              });
            }}
          />
        </Box>
      ),
    },
    mainMenu: {
      selector: '[data-component="MainMenu"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step6">
            {tourText.MainMenu.text}
          </Typography>
          <NavButtons goTo={goTo} step={step} />
        </Box>
      ),
    },
    doorsPanel: {
      selector: '[data-item="Doors"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step7">
            {tourText.DoorPanel.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.Doors,
              });
              doorSpotlight && doorSpotlight();
            }}
          />
        </Box>
      ),
    },
    doorTab: {
      selector: '[data-name="main_door"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step8">
            {tourText.DoorTab.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.MainMenu,
              });
            }}
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.MainMenu,
              });
            }}
          />
        </Box>
      ),
    },
    commandsPanel: {
      selector: '[data-item="Commands"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step9">
            {tourText.CommandsPanel.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.Commands,
              });
            }}
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.Doors,
              });
            }}
          />
        </Box>
      ),
    },
    loopRequest: {
      selector: '[data-component="LoopForm"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step10">
            {tourText.LoopRequest.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(false, false, false, false)
            }
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.MainMenu,
              });
            }}
          />
        </Box>
      ),
    },
    settingsButton: {
      selector: '[id="show-settings-btn"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step11">
            {tourText.Settings.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(true, false, false, false)
            }
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.Commands,
              });
            }}
          />
        </Box>
      ),
    },
    trajAnim: {
      selector: '.MuiDrawer-paper',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step12">
            {tourText.TrajAnims.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() =>
              showSettingsOmniPanelHelpClearSpotlight(false, false, false, true)
            }
            handleBackClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, false, false, true);
            }}
          />
        </Box>
      ),
    },
    helpButton: {
      selector: '[id="show-help-btn"]',
      content: ({ goTo, step }) => (
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step13">
            {tourText.HelpButton.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, false, true, true);
            }}
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
        <Box data-testid="stepBox">
          <Typography variant="h6" data-testid="step14">
            {tourText.HelpDrawer.text}
          </Typography>
          <NavButtons
            goTo={goTo}
            step={step}
            handleNextClick={() => {
              showSettingsOmniPanelHelpClearSpotlight(false, true, false, false);
              dispatchMenu({
                type: MainMenuActionType.CURRENT_VIEW,
                payload: OmniPanelViewIndex.MainMenu,
              });
              dispatchMenu({
                type: MainMenuActionType.TOUR_STATE,
                payload: false,
              });
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
