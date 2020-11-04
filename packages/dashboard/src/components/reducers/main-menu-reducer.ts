import { Settings } from '../../settings';
import { LoadingScreenProps } from '../loading-screen';

type ActionFormat<T, K = undefined> = {
  type: T;
  payload: K;
};

export enum MainMenuActionType {
  SHOW_OMNIPANEL = 'showOmniPanel',
  TOGGLE_OMNIPANEL = 'toggleOmnipanel',
  CURRENT_VIEW = 'currentView',
  LOADING = 'loading',
  SHOW_SETTINGS = 'showSettings',
  SETTINGS = 'settings',
  SHOW_HELP = 'showHelp',
  TOUR_STATE = 'tourState',
}

type Action =
  | ActionFormat<'showOmniPanel', boolean>
  | ActionFormat<'currentView', number>
  | ActionFormat<'loading', LoadingScreenProps | null>
  | ActionFormat<'showSettings', boolean>
  | ActionFormat<'settings', Settings>
  | ActionFormat<'showHelp', boolean>
  | ActionFormat<'toggleOmnipanel'>
  | ActionFormat<'tourState', boolean>;

export type MainMenuState = {
  [MainMenuActionType.SHOW_OMNIPANEL]: boolean;
  [MainMenuActionType.CURRENT_VIEW]: number;
  [MainMenuActionType.LOADING]: LoadingScreenProps | null;
  [MainMenuActionType.SHOW_SETTINGS]: boolean;
  [MainMenuActionType.SETTINGS]: Settings;
  [MainMenuActionType.SHOW_HELP]: boolean;
  [MainMenuActionType.TOUR_STATE]: boolean;
};

export const mainMenuReducer = (state: MainMenuState, action: Action): MainMenuState => {
  // get the list of Enums values
  switch (action.type) {
    case MainMenuActionType.SHOW_OMNIPANEL:
      return { ...state, [MainMenuActionType.SHOW_OMNIPANEL]: action.payload };
    case MainMenuActionType.TOGGLE_OMNIPANEL:
      return { ...state, showOmniPanel: !state.showOmniPanel };
    case MainMenuActionType.CURRENT_VIEW:
      return { ...state, [MainMenuActionType.CURRENT_VIEW]: action.payload };
    case MainMenuActionType.LOADING:
      return { ...state, [MainMenuActionType.LOADING]: action.payload };
    case MainMenuActionType.SHOW_SETTINGS:
      return { ...state, [MainMenuActionType.SHOW_SETTINGS]: action.payload };
    case MainMenuActionType.SETTINGS:
      return { ...state, [MainMenuActionType.SETTINGS]: action.payload };
    case MainMenuActionType.SHOW_HELP:
      return { ...state, [MainMenuActionType.SHOW_HELP]: action.payload };
    case MainMenuActionType.TOUR_STATE:
      return { ...state, tourState: action.payload };
    default:
      console.error('Unexpected action');
      return state;
  }
};

// const [showOmniPanel, setShowOmniPanel] = React.useState(true);
// const [currentView, setCurrentView] = React.useState(OmniPanelViewIndex.MainMenu);
// const [loading, setLoading] = React.useState<LoadingScreenProps | null>({
//   caption: 'Connecting to api server...',
// });

// const [showSettings, setShowSettings] = React.useState(false);
// const [settings, setSettings] = React.useState<Settings>(() => loadSettings());

// const [showHelp, setShowHelp] = React.useState(false);
