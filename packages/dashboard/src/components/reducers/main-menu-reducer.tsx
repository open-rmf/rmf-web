import React from 'react';
import { StackNavigator } from 'react-components';
import { Settings } from '../../settings';
import { OmniPanelViewIndex } from '../dashboard';
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
  ShowNotifications = 'showNotifications',
  PopView = 'popView',
  PushView = 'pushView',
  ResetView = 'resetView',
  ToggleHelp = 'toggleHelp',
  ToggleHotkeys = 'toggleHotkeys',
  ToggleOmnipanel = 'toggleOmnipanel',
  ToggleSettings = 'toggleSettings',
  TourState = 'tourState',
  CountNotfications = 'countNotification',
  UpdateNotifications = 'updateNotifications',
}

export type MainMenuState = {
  [MainMenuActionType.CurrentView]: number;
  [MainMenuActionType.Loading]: LoadingScreenProps | null;
  [MainMenuActionType.Settings]: Settings;
  [MainMenuActionType.ShowHelp]: boolean;
  [MainMenuActionType.ShowHotkeysDialog]: boolean;
  [MainMenuActionType.ShowOmniPanel]: boolean;
  [MainMenuActionType.ShowSettings]: boolean;
  [MainMenuActionType.ShowNotifications]: boolean;
  [MainMenuActionType.CountNotfications]: number;
  [MainMenuActionType.UpdateNotifications]: { [key: string]: string }[];
  stackNavigator: StackNavigator<OmniPanelViewIndex>;
};

export type MainMenuAction =
  | MainMenuActionFormat<'currentView', MainMenuState['currentView']>
  | MainMenuActionFormat<'loading', MainMenuState['loading']>
  | MainMenuActionFormat<'settings', MainMenuState['settings']>
  | MainMenuActionFormat<'showHelp', MainMenuState['showHelp']>
  | MainMenuActionFormat<'showHotkeysDialog', MainMenuState['showHotkeysDialog']>
  | MainMenuActionFormat<'showOmniPanel', MainMenuState['showOmniPanel']>
  | MainMenuActionFormat<'showSettings', MainMenuState['showSettings']>
  | MainMenuActionFormat<'showNotifications', MainMenuState['showNotifications']>
  | MainMenuActionFormat<'popView'>
  | MainMenuActionFormat<'pushView', MainMenuState['currentView']>
  | MainMenuActionFormat<'resetView'>
  | MainMenuActionFormat<'toggleHelp'>
  | MainMenuActionFormat<'toggleHotkeys'>
  | MainMenuActionFormat<'toggleOmnipanel'>
  | MainMenuActionFormat<'toggleSettings'>
  | MainMenuActionFormat<'countNotification', MainMenuState['countNotification']>
  | MainMenuActionFormat<'updateNotifications', MainMenuState['updateNotifications']>;

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
    case MainMenuActionType.ShowNotifications:
      return { ...state, [MainMenuActionType.ShowNotifications]: action.payload };
    case MainMenuActionType.PopView:
      return { ...state, [MainMenuActionType.CurrentView]: state.stackNavigator.pop() };
    case MainMenuActionType.PushView:
      state.stackNavigator.push(action.payload);
      return { ...state, [MainMenuActionType.CurrentView]: action.payload };
    case MainMenuActionType.ResetView:
      state.stackNavigator.reset();
      return state;
    case MainMenuActionType.ToggleOmnipanel:
      return { ...state, showOmniPanel: !state.showOmniPanel };
    case MainMenuActionType.ToggleSettings:
      return { ...state, showSettings: !state.showSettings };
    case MainMenuActionType.ToggleHelp:
      return { ...state, showHelp: !state.showHelp };
    case MainMenuActionType.ToggleHotkeys:
      return { ...state, showHotkeysDialog: !state.showHotkeysDialog };
    case MainMenuActionType.CountNotfications:
      return { ...state, [MainMenuActionType.CountNotfications]: action.payload };
    case MainMenuActionType.UpdateNotifications:
      return { ...state, [MainMenuActionType.UpdateNotifications]: action.payload };
    default:
      console.error('Unexpected action');
      return state;
  }
};

export interface ReducerMainMenuDispatch {
  popView: () => void;
  pushView: (payload: MainMenuState['currentView']) => void;
  resetView: () => void;
  setCurrentView: (payload: MainMenuState['currentView']) => void;
  setLoading: (payload: MainMenuState['loading']) => void;
  setSettings: (payload: MainMenuState['settings']) => void;
  setShowHelp: (payload: MainMenuState['showHelp']) => void;
  setShowHotkeysDialog: (payload: MainMenuState['showHotkeysDialog']) => void;
  setShowOmniPanel: (payload: MainMenuState['showOmniPanel']) => void;
  setShowSettings: (payload: MainMenuState['showSettings']) => void;
  setShowNotifications: (payload: MainMenuState['showNotifications']) => void;
  toggleHelp: () => void;
  toggleHotkeys: () => void;
  toggleOmnipanel: () => void;
  toggleSettings: () => void;
  countNotifications: (payload: MainMenuState['countNotification']) => void;
  updateNotifications: (payload: MainMenuState['updateNotifications']) => void;
}
export interface ReducerMainMenuProps {
  state: MainMenuState;
  dispatch: ReducerMainMenuDispatch;
}

export const useMainMenuReducer = (initialValue: MainMenuState): ReducerMainMenuProps => {
  const [_state, _dispatch] = React.useReducer(mainMenuReducer, initialValue);
  // We add a useMemo here because React identifies that the state and dispatch props are always
  // changing, which causes a huge performance issue.
  const state = React.useMemo(() => _state, [_state]);
  const dispatch: ReducerMainMenuDispatch = React.useMemo(() => {
    return {
      popView: () => _dispatch({ type: MainMenuActionType.PopView }),
      pushView: (payload) => _dispatch({ type: MainMenuActionType.PushView, payload: payload }),
      resetView: () => _dispatch({ type: MainMenuActionType.ResetView }),
      setCurrentView: (payload) =>
        _dispatch({ type: MainMenuActionType.CurrentView, payload: payload }),
      setLoading: (payload) => _dispatch({ type: MainMenuActionType.Loading, payload: payload }),
      setSettings: (payload) => _dispatch({ type: MainMenuActionType.Settings, payload: payload }),
      setShowHelp: (payload) => _dispatch({ type: MainMenuActionType.ShowHelp, payload: payload }),
      setShowHotkeysDialog: (payload) =>
        _dispatch({ type: MainMenuActionType.ShowHotkeysDialog, payload: payload }),
      setShowOmniPanel: (payload) =>
        _dispatch({ type: MainMenuActionType.ShowOmniPanel, payload: payload }),
      setShowSettings: (payload) =>
        _dispatch({ type: MainMenuActionType.ShowSettings, payload: payload }),
      setShowNotifications: (payload) =>
        _dispatch({ type: MainMenuActionType.ShowNotifications, payload }),
      toggleHelp: () => _dispatch({ type: MainMenuActionType.ToggleHelp }),
      toggleHotkeys: () => _dispatch({ type: MainMenuActionType.ToggleHotkeys }),
      toggleOmnipanel: () => _dispatch({ type: MainMenuActionType.ToggleOmnipanel }),
      toggleSettings: () => _dispatch({ type: MainMenuActionType.ToggleSettings }),
      countNotifications: (payload) =>
        _dispatch({ type: MainMenuActionType.CountNotfications, payload }),
      updateNotifications: (payload) =>
        _dispatch({ type: MainMenuActionType.UpdateNotifications, payload }),
    } as ReducerMainMenuDispatch;
  }, []);
  return {
    state,
    dispatch,
  };
};
