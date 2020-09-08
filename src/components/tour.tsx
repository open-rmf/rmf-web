import React from 'react';
import { Typography, makeStyles } from '@material-ui/core';
import {
  NavigateNext as NavigateNextIcon,
  NavigateBefore as NavigateBeforeIcon,
} from '@material-ui/icons';
import { SpotlightValue } from './spotlight-value';
import Tour from 'reactour';
import { createTourSteps } from './tour-manager';
import { OmniPanelViewIndex } from './app';

export interface DashboardTourProps {
  tourProps: {
    tourState: boolean;
    setTourState: React.Dispatch<React.SetStateAction<boolean>>;
    OmniPanelViewIndex: any;
    setTourShowOmniPanel: (view: OmniPanelViewIndex) => void;
    setTourSettingsAndOmniPanel: (
      isSettingsVisible: boolean,
      isOmniPanelVisible: boolean,
      clearSpotlight?: boolean | undefined,
    ) => void;
    doorSpotlight: SpotlightValue<string> | undefined;
    setDoorSpotlight: React.Dispatch<React.SetStateAction<SpotlightValue<string> | undefined>>;
  };
}

export default function DashboardTour(props: DashboardTourProps): React.ReactElement {
  const useStyles = makeStyles(theme => ({
    contained: {
      color: theme.palette.info.contrastText,
      backgroundColor: theme.palette.primary.main,
      borderRadius: '5px',
      boxShadow: theme.shadows[2],
      '&:hover': {
        backgroundColor: theme.palette.grey.A100,
        boxShadow: theme.shadows[4],
        '@media (hover: none)': {
          boxShadow: theme.shadows[2],
          backgroundColor: theme.palette.grey[300],
        },
      },
    },
  }));
  const classes = useStyles();
  const {
    tourProps: {
      tourState,
      setTourState,
      setTourSettingsAndOmniPanel,
      setTourShowOmniPanel,
      OmniPanelViewIndex,
      doorSpotlight,
      setDoorSpotlight,
    },
  } = props;

  const tourFunctions = {
    setTourShowOmniPanel,
    setTourSettingsAndOmniPanel,
    OmniPanelViewIndex,
    doorSpotlight,
    setDoorSpotlight,
  };
  const { tourSteps, theme } = createTourSteps(tourFunctions);

  const lastStepNextButton = (
    <Typography className={classes.contained}>Start using RoMi</Typography>
  );

  return (
    <Tour
      steps={tourSteps}
      isOpen={tourState}
      onRequestClose={() => {
        setTourState(false);
        setTourShowOmniPanel(OmniPanelViewIndex.MainMenu);
      }}
      badgeContent={(curr, tot) => `${curr} of ${tot}`}
      accentColor={theme.palette.primary.main}
      rounded={5}
      showNavigationNumber={false}
      nextButton={<NavigateNextIcon />}
      prevButton={<NavigateBeforeIcon />}
      lastStepNextButton={lastStepNextButton}
    />
  );
}
