import React from 'react';
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
    setTourState,
    setTourShowOmniPanel,
    setTourSettingsAndOmniPanel,
    OmniPanelViewIndex,
    doorSpotlight,
    setDoorSpotlight,
  };
  const { tourSteps, theme } = createTourSteps(tourFunctions);

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
      showNavigation={false}
      showButtons={false}
      closeWithMask={false}
      startAt={0}
    />
  );
}
