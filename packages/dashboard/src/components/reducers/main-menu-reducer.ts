import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Settings } from '../../settings';
import { LoadingScreenProps } from '../loading-screen';

type ActionFormat<T, K> = {
  type: T;
  payload: K;
};

export enum MainMenuActionType {
  SHOW_OMNIPANEL = 'showOmniPanel',
  CURRENT_VIEW = 'currentView',
  LOADING = 'loading',
  SHOW_SETTINGS = 'showSettings',
  SETTINGS = 'settings',
  SHOW_HELP = 'showHelp',
}

type Action =
  | ActionFormat<'showOmniPanel', boolean>
  | ActionFormat<'currentView', number>
  | ActionFormat<'loading', LoadingScreenProps | null>
  | ActionFormat<'showSettings', boolean>
  | ActionFormat<'settings', Settings>
  | ActionFormat<'showHelp', boolean>;

export type MainMenuState = {
  [MainMenuActionType.SHOW_OMNIPANEL]: boolean;
  [MainMenuActionType.CURRENT_VIEW]: number;
  [MainMenuActionType.LOADING]: LoadingScreenProps | null;
  [MainMenuActionType.SHOW_SETTINGS]: boolean;
  [MainMenuActionType.SETTINGS]: Settings;
  [MainMenuActionType.SHOW_HELP]: boolean;
};

export const mainMenuReducer = (state: MainMenuState, action: Action): MainMenuState => {
  if (!(action.type in Object.keys(MainMenuActionType))) {
    throw new Error('Unexpected action');
  }
  return { ...state, [action.type]: action.payload };
  // if (action.type)
  //   // get the list of Enums values
  //   switch (action.type) {
  //     case MainMenuActionType.SHOW_OMNIPANEL:
  //       return { ...state, [MainMenuActionType.SHOW_OMNIPANEL]: action.payload };
  //     case MainMenuActionType.CURRENT_VIEW:
  //       return { ...state, currentView: action.payload };
  //     case MainMenuActionType.LOADING:
  //       return { ...state, loading: action.payload };
  //     case MainMenuActionType.SHOW_SETTINGS:
  //       return { ...state, showSettings: action.payload };
  //     case MainMenuActionType.SETTINGS:
  //       return { ...state, settings: action.payload };
  //     case MainMenuActionType.SHOW_HELP:
  //       return { ...state, showHelp: action.payload };
  //     default:
  //       throw new Error('Unexpected action');
  //   }
};

// const [showOmniPanel, setShowOmniPanel] = React.useState(true);
// const [currentView, setCurrentView] = React.useState(OmniPanelViewIndex.MainMenu);
// const [loading, setLoading] = React.useState<LoadingScreenProps | null>({
//   caption: 'Connecting to api server...',
// });

// const [showSettings, setShowSettings] = React.useState(false);
// const [settings, setSettings] = React.useState<Settings>(() => loadSettings());

// const [showHelp, setShowHelp] = React.useState(false);
