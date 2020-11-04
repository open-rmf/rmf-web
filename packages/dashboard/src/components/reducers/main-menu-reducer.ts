import { Settings } from '../../settings';
import { LoadingScreenProps } from '../loading-screen';

type ActionFormat<T, K = undefined> = {
  type: T;
  payload: K;
};

export enum MainMenuActionType {
  CURRENT_VIEW = 'currentView',
  LOADING = 'loading',
  SETTINGS = 'settings',
  SHOW_HELP = 'showHelp',
  SHOW_HOTKEYS_DIALOG = 'showHotkeysDialog',
  SHOW_OMNIPANEL = 'showOmniPanel',
  SHOW_SETTINGS = 'showSettings',
  TOGGLE_HELP = 'toggleHelp',
  TOGGLE_HOTKEYS = 'toggleHotkeys',
  TOGGLE_OMNIPANEL = 'toggleOmnipanel',
  TOGGLE_SETTINGS = 'toggleSettings',
  TOUR_STATE = 'tourState',
}

type Action =
  | ActionFormat<'currentView', number>
  | ActionFormat<'loading', LoadingScreenProps | null>
  | ActionFormat<'settings', Settings>
  | ActionFormat<'showHelp', boolean>
  | ActionFormat<'showHotkeysDialog', boolean>
  | ActionFormat<'showOmniPanel', boolean>
  | ActionFormat<'showSettings', boolean>
  | ActionFormat<'toggleHelp', boolean>
  | ActionFormat<'toggleHotkeys', boolean>
  | ActionFormat<'toggleOmnipanel'>
  | ActionFormat<'toggleSettings', boolean>
  | ActionFormat<'tourState', boolean>;

export type MainMenuState = {
  [MainMenuActionType.CURRENT_VIEW]: number;
  [MainMenuActionType.LOADING]: LoadingScreenProps | null;
  [MainMenuActionType.SETTINGS]: Settings;
  [MainMenuActionType.SHOW_HELP]: boolean;
  [MainMenuActionType.SHOW_HOTKEYS_DIALOG]: boolean;
  [MainMenuActionType.SHOW_OMNIPANEL]: boolean;
  [MainMenuActionType.SHOW_SETTINGS]: boolean;
  [MainMenuActionType.TOUR_STATE]: boolean;
};

export const mainMenuReducer = (state: MainMenuState, action: Action): MainMenuState => {
  switch (action.type) {
    case MainMenuActionType.CURRENT_VIEW:
      return { ...state, [MainMenuActionType.CURRENT_VIEW]: action.payload };
    case MainMenuActionType.LOADING:
      return { ...state, [MainMenuActionType.LOADING]: action.payload };
    case MainMenuActionType.SETTINGS:
      return { ...state, [MainMenuActionType.SETTINGS]: action.payload };
    case MainMenuActionType.SHOW_OMNIPANEL:
      return { ...state, [MainMenuActionType.SHOW_OMNIPANEL]: action.payload };
    case MainMenuActionType.SHOW_SETTINGS:
      return { ...state, [MainMenuActionType.SHOW_SETTINGS]: action.payload };
    case MainMenuActionType.SHOW_HOTKEYS_DIALOG:
      return { ...state, [MainMenuActionType.SHOW_HOTKEYS_DIALOG]: action.payload };
    case MainMenuActionType.SHOW_HELP:
      return { ...state, [MainMenuActionType.SHOW_HELP]: action.payload };
    case MainMenuActionType.TOGGLE_OMNIPANEL:
      return { ...state, showOmniPanel: !state.showOmniPanel };
    case MainMenuActionType.TOGGLE_SETTINGS:
      return { ...state, showSettings: !state.showSettings };
    case MainMenuActionType.TOGGLE_HELP:
      return { ...state, showHelp: !state.showHelp };
    case MainMenuActionType.TOGGLE_HOTKEYS:
      return { ...state, showHotkeysDialog: !state.showHotkeysDialog };
    case MainMenuActionType.TOUR_STATE:
      return { ...state, tourState: action.payload };
    default:
      console.error('Unexpected action');
      return state;
  }
};
