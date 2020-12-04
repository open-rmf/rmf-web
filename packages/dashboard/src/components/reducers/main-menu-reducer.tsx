import React from 'react';
import { Settings } from '../../settings';
import { LoadingScreenProps } from '../loading-screen';

type MainMenuActionFormat<T, K = void> = K extends void
  ? {
      type: T;
    }
  : {
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

export type MainMenuAction =
  | MainMenuActionFormat<'currentView', MainMenuState['currentView']>
  | MainMenuActionFormat<'loading', MainMenuState['loading']>
  | MainMenuActionFormat<'settings', MainMenuState['settings']>
  | MainMenuActionFormat<'showHelp', MainMenuState['showHelp']>
  | MainMenuActionFormat<'showHotkeysDialog', MainMenuState['showHotkeysDialog']>
  | MainMenuActionFormat<'showOmniPanel', MainMenuState['showOmniPanel']>
  | MainMenuActionFormat<'showSettings', MainMenuState['showSettings']>
  | MainMenuActionFormat<'toggleHelp'>
  | Partial<MainMenuActionFormat<'toggleHotkeys'>>
  | Partial<MainMenuActionFormat<'toggleOmnipanel'>>
  | Partial<MainMenuActionFormat<'toggleSettings'>>
  | MainMenuActionFormat<'tourState', MainMenuState['tourState']>;

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

export interface ReducerMainMenuProps extends MainMenuState {
  setCurrentView: (payload: MainMenuState['currentView']) => void;
  setLoading: (payload: MainMenuState['loading']) => void;
  setSettings: (payload: MainMenuState['settings']) => void;
  setShowHelp: (payload: MainMenuState['showHelp']) => void;
  setShowHotkeysDialog: (payload: MainMenuState['showHotkeysDialog']) => void;
  setShowOmniPanel: (payload: MainMenuState['showOmniPanel']) => void;
  setShowSettings: (payload: MainMenuState['showSettings']) => void;
  toggleHelp: () => void;
  toggleHotkeys: () => void;
  toggleOmnipanel: () => void;
  toggleSettings: () => void;
  setTourState: (payload: MainMenuState['tourState']) => void;
}

export const useMainMenuReducer = (initialValue: MainMenuState): ReducerMainMenuProps => {
  const [state, dispatch] = React.useReducer(mainMenuReducer, initialValue);
  const {
    currentView,
    loading,
    settings,
    showHelp,
    showHotkeysDialog,
    showOmniPanel,
    showSettings,
    tourState,
  } = state;
  return {
    currentView,
    loading,
    settings,
    showHelp,
    showHotkeysDialog,
    showOmniPanel,
    showSettings,
    tourState,
    setCurrentView: (payload) =>
      dispatch({ type: MainMenuActionType.CurrentView, payload: payload }),
    setLoading: (payload) => dispatch({ type: MainMenuActionType.Loading, payload: payload }),
    setSettings: (payload) => dispatch({ type: MainMenuActionType.Settings, payload: payload }),
    setShowHelp: (payload) => dispatch({ type: MainMenuActionType.ShowHelp, payload: payload }),
    setShowHotkeysDialog: (payload) =>
      dispatch({ type: MainMenuActionType.ShowHotkeysDialog, payload: payload }),
    setShowOmniPanel: (payload) =>
      dispatch({ type: MainMenuActionType.ShowOmniPanel, payload: payload }),
    setShowSettings: (payload) =>
      dispatch({ type: MainMenuActionType.ShowSettings, payload: payload }),
    toggleHelp: () => dispatch({ type: MainMenuActionType.ToggleHelp }),
    toggleHotkeys: () => dispatch({ type: MainMenuActionType.ToggleHotkeys }),
    toggleOmnipanel: () => dispatch({ type: MainMenuActionType.ToggleOmnipanel }),
    toggleSettings: () => dispatch({ type: MainMenuActionType.ToggleSettings }),
    setTourState: (payload) => dispatch({ type: MainMenuActionType.TourState, payload: payload }),
  };
};
