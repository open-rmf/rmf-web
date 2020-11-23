import { Settings } from '../../settings';
import { LoadingScreenProps } from '../loading-screen';

type MainMenuActionFormat<T, K = undefined> = {
  type: T;
  payload: K;
};

export enum MainMenuActionType {
  CurrentView = 'currentView',
  Loading = 'loading',
  Settings = 'settings',
  ShowHelp = 'showHelp',
  ShowHotkeysDialog = 'showHotkeysDialog',
  ShowOmniPanel = 'showOmniPanel',
  ShowSettings = 'showSettings',
  ToggleHelp = 'toggleHelp',
  ToggleHotkeys = 'toggleHotkeys',
  ToggleOmnipanel = 'toggleOmnipanel',
  ToggleSettings = 'toggleSettings',
  TourState = 'tourState',
}

export type MainMenuAction =
  | MainMenuActionFormat<'currentView', number>
  | MainMenuActionFormat<'loading', LoadingScreenProps | null>
  | MainMenuActionFormat<'settings', Settings>
  | MainMenuActionFormat<'showHelp', boolean>
  | MainMenuActionFormat<'showHotkeysDialog', boolean>
  | MainMenuActionFormat<'showOmniPanel', boolean>
  | MainMenuActionFormat<'showSettings', boolean>
  | Partial<MainMenuActionFormat<'toggleHelp'>>
  | Partial<MainMenuActionFormat<'toggleHotkeys'>>
  | Partial<MainMenuActionFormat<'toggleOmnipanel'>>
  | Partial<MainMenuActionFormat<'toggleSettings'>>
  | MainMenuActionFormat<'tourState', boolean>;

export type MainMenuState = {
  [MainMenuActionType.CurrentView]: number;
  [MainMenuActionType.Loading]: LoadingScreenProps | null;
  [MainMenuActionType.Settings]: Settings;
  [MainMenuActionType.ShowHelp]: boolean;
  [MainMenuActionType.ShowHotkeysDialog]: boolean;
  [MainMenuActionType.ShowOmniPanel]: boolean;
  [MainMenuActionType.ShowSettings]: boolean;
  [MainMenuActionType.TourState]: boolean;
};

export const mainMenuReducer = (state: MainMenuState, action: MainMenuAction): MainMenuState => {
  switch (action.type) {
    case MainMenuActionType.CurrentView:
      return { ...state, [MainMenuActionType.CurrentView]: action.payload };
    case MainMenuActionType.Loading:
      return { ...state, [MainMenuActionType.Loading]: action.payload };
    case MainMenuActionType.Settings:
      return { ...state, [MainMenuActionType.Settings]: action.payload };
    case MainMenuActionType.ShowOmniPanel:
      return { ...state, [MainMenuActionType.ShowOmniPanel]: action.payload };
    case MainMenuActionType.ShowSettings:
      return { ...state, [MainMenuActionType.ShowSettings]: action.payload };
    case MainMenuActionType.ShowHotkeysDialog:
      return { ...state, [MainMenuActionType.ShowHotkeysDialog]: action.payload };
    case MainMenuActionType.ShowHelp:
      return { ...state, [MainMenuActionType.ShowHelp]: action.payload };
    case MainMenuActionType.ToggleOmnipanel:
      return { ...state, showOmniPanel: !state.showOmniPanel };
    case MainMenuActionType.ToggleSettings:
      return { ...state, showSettings: !state.showSettings };
    case MainMenuActionType.ToggleHelp:
      return { ...state, showHelp: !state.showHelp };
    case MainMenuActionType.ToggleHotkeys:
      return { ...state, showHotkeysDialog: !state.showHotkeysDialog };
    case MainMenuActionType.TourState:
      return { ...state, tourState: action.payload };
    default:
      console.error('Unexpected action');
      return state;
  }
};
