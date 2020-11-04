import React from 'react';
import Tour from 'reactour';
import { OmniPanelViewIndex } from '../dashboard';
import { MainMenuActionType, MainMenuState } from '../reducers/main-menu-reducer';
import { createTourSteps } from './tour-manager';

export interface DashboardTourProps {
  tourProps: {
    dispatchMenu: any;
    stateMenu: MainMenuState;
    clearSpotlights: () => void;
    doorSpotlight?: () => void;
  };
}

export const DashboardTour = React.memo(
  (props: DashboardTourProps): React.ReactElement => {
    const {
      tourProps: { dispatchMenu, stateMenu, clearSpotlights, doorSpotlight },
    } = props;

    const tourFunctions = {
      dispatchMenu,
      clearSpotlights,
      doorSpotlight,
    };
    const { tourSteps, theme } = createTourSteps(tourFunctions);

    const [update, setUpdate] = React.useState(0);

    /**
     * Qn: Best solution to handle animations?
     *
     * 1. update the tour on an interval.
     * 2. set a timeout between each type, this has the downside that it would look laggy and
     * would also breaks when the animation length changes.
     * 3. Listen for all animation end callback and update the tour. (current solution)
     */
    React.useEffect(() => {
      const listener = () => setUpdate((prev) => prev + 1);
      // listen on specific component instead of the whole DOM?
      document.addEventListener('transitionend', listener);
      return () => document.removeEventListener('transitionend', listener);
    }, []);

    return (
      <Tour
        steps={tourSteps}
        isOpen={stateMenu.tourState}
        onRequestClose={() => {
          localStorage.setItem('tourComplete', 'true');
          dispatchMenu({ type: MainMenuActionType.TOUR_STATE, payload: false });
          dispatchMenu({ type: MainMenuActionType.SHOW_SETTINGS, payload: false });
          dispatchMenu({ type: MainMenuActionType.SHOW_HELP, payload: false });
          dispatchMenu({ type: MainMenuActionType.SHOW_OMNIPANEL, payload: true });
          dispatchMenu({
            type: MainMenuActionType.CURRENT_VIEW,
            payload: OmniPanelViewIndex.MainMenu,
          });
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
        update={update.toString()}
      />
    );
  },
);

export default DashboardTour;
