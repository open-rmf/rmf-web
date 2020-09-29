import React from 'react';
import { SpotlightValue } from '../spotlight-value';
import Tour from 'reactour';
import { createTourSteps } from './tour-manager';
import { OmniPanelViewIndex } from '../dashboard';

export interface DashboardTourProps {
  tourProps: {
    tourState: boolean;
    setTourState: React.Dispatch<React.SetStateAction<boolean>>;
    setShowSettings: React.Dispatch<React.SetStateAction<boolean>>;
    setShowOmniPanel: React.Dispatch<React.SetStateAction<boolean>>;
    setShowHelp: React.Dispatch<React.SetStateAction<boolean>>;
    clearSpotlights: () => void;
    setCurrentView: React.Dispatch<React.SetStateAction<OmniPanelViewIndex>>;
    doorSpotlight: SpotlightValue<string> | undefined;
    setDoorSpotlight: React.Dispatch<React.SetStateAction<SpotlightValue<string> | undefined>>;
  };
}

export const DashboardTour = React.memo(
  (props: DashboardTourProps): React.ReactElement => {
    const {
      tourProps: {
        tourState,
        setTourState,
        setShowSettings,
        setShowOmniPanel,
        setShowHelp,
        clearSpotlights,
        setCurrentView,
        doorSpotlight,
        setDoorSpotlight,
      },
    } = props;

    const tourFunctions = {
      setTourState,
      setShowSettings,
      setShowOmniPanel,
      setShowHelp,
      clearSpotlights,
      setCurrentView,
      doorSpotlight,
      setDoorSpotlight,
    };
    const { tourSteps, theme } = createTourSteps(tourFunctions);

    return (
      <Tour
        steps={tourSteps}
        isOpen={tourState}
        onRequestClose={() => {
          localStorage.setItem('tourComplete', 'true');
          setTourState(false);
          setShowSettings(false);
          setShowHelp(false);
          setShowOmniPanel(true);
          setCurrentView(OmniPanelViewIndex.MainMenu);
        }}
        badgeContent={(curr, tot) => `${curr} of ${tot}`}
        accentColor={theme.palette.primary.main}
        rounded={5}
        showNavigationNumber={false}
        showNavigation={false}
        disableKeyboardNavigation={['right', 'left']}
        showButtons={false}
        closeWithMask={false}
        startAt={0}
      />
    );
  },
);

export default DashboardTour;
