import { Notification } from '../index';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

export interface ItemSummaryState {
  operational: number;
  outOfOrder: number;
  idle?: number;
  charging?: number;
}

export interface ItemSummary {
  item: string;
  summary: ItemSummaryState;
  outOfOrder: string[];
}

export interface MainMenuItemStateProps {
  itemSummary: ItemSummary;
  handleClick: () => void;
}

export interface MainMenuAlertProps {
  notifications: Notification[];
}

export interface MainMenuTaskStateProps {
  tasks: RomiCore.TaskSummary[];
}

export interface TaskSummaryState {
  active: number;
  finish: number;
  failed: number;
  queued: number;
}

export interface MainMenuBannerProps {
  bannerUrl: string;
  isError: boolean;
}

export interface SpoiltItem {
  summary: string;
  errorMessage?: string;
}

export interface MainMenuSpoiltItemsProps {
  spoiltItems: SpoiltItem[];
  errorMessage?: string;
}
