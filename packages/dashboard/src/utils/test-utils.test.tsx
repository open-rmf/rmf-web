import '@testing-library/dom';

import { render as render_, RenderOptions as RenderOptions_ } from '@testing-library/react';
import {
  AdminApi,
  AlertRequest,
  AlertResponse,
  AlertsApi,
  BeaconsApi,
  BeaconState,
  BuildingApi,
  BuildingMap,
  DefaultApi,
  DeliveryAlert,
  DeliveryAlertsApi,
  Dispenser,
  DispensersApi,
  DispenserState,
  Door,
  DoorsApi,
  DoorState,
  FleetsApi,
  FleetState,
  Ingestor,
  IngestorsApi,
  IngestorState,
  Lift,
  LiftsApi,
  LiftState,
  TasksApi,
  TaskStateOutput,
} from 'api-client';
import axios from 'axios';
import React from 'react';
import { Subject } from 'rxjs';
import { vi } from 'vitest';

import { AppController, AppControllerProvider } from '../hooks/use-app-controller';
import { AuthenticatorProvider } from '../hooks/use-authenticator';
import { Resources, ResourcesProvider } from '../hooks/use-resources';
import { RmfApiProvider } from '../hooks/use-rmf-api';
import { SettingsProvider } from '../hooks/use-settings';
import { TaskRegistry, TaskRegistryProvider } from '../hooks/use-task-registry';
import { UserProfileProvider } from '../hooks/use-user-profile';
import { UserProfile } from '../services/authenticator';
import { RmfApi } from '../services/rmf-api';
import { Settings } from '../services/settings';
import StubAuthenticator from '../services/stub-authenticator';

export const superUser: UserProfile = {
  user: {
    username: 'test',
    is_admin: true,
    roles: [],
  },
  permissions: [],
};

export function makeMockAppController(): AppController {
  return {
    updateSettings: vi.fn(),
    showAlert: vi.fn(),
    setExtraAppbarItems: vi.fn(),
  };
}

export class MockRmfApi implements RmfApi {
  axiosInst = axios.create({
    adapter: (config) =>
      Promise.reject({ config, data: null, headers: {}, status: 500, statusText: '' }),
  });
  beaconsApi = new BeaconsApi(undefined, undefined, this.axiosInst);
  buildingApi = new BuildingApi(undefined, undefined, this.axiosInst);
  defaultApi = new DefaultApi(undefined, undefined, this.axiosInst);
  doorsApi = new DoorsApi(undefined, undefined, this.axiosInst);
  liftsApi = new LiftsApi(undefined, undefined, this.axiosInst);
  dispensersApi = new DispensersApi(undefined, undefined, this.axiosInst);
  ingestorsApi = new IngestorsApi(undefined, undefined, this.axiosInst);
  fleetsApi = new FleetsApi(undefined, undefined, this.axiosInst);
  tasksApi = new TasksApi(undefined, undefined, this.axiosInst);
  alertsApi = new AlertsApi(undefined, undefined, this.axiosInst);
  adminApi = new AdminApi(undefined, undefined, this.axiosInst);
  deliveryAlertsApi = new DeliveryAlertsApi(undefined, undefined, this.axiosInst);
  negotiationStatusManager = undefined;
  trajectoryManager = undefined;
  buildingMapObs = new Subject<BuildingMap>();
  beaconsObsStore = new Subject<BeaconState>();
  doorsObs = new Subject<Door[]>();
  doorStateObsStore: Record<string, Subject<DoorState>> = {};
  getDoorStateObs(name: string) {
    return this._cacheObs(this.doorStateObsStore, name);
  }
  liftsObs = new Subject<Lift[]>();
  liftStateObsStore: Record<string, Subject<LiftState>> = {};
  getLiftStateObs(name: string) {
    return this._cacheObs(this.liftStateObsStore, name);
  }
  dispensersObs = new Subject<Dispenser[]>();
  dispenserStateObsStore: Record<string, Subject<DispenserState>> = {};
  getDispenserStateObs(guid: string) {
    return this._cacheObs(this.dispenserStateObsStore, guid);
  }
  ingestorsObs = new Subject<Ingestor[]>();
  ingestorStateObsStore: Record<string, Subject<IngestorState>> = {};
  getIngestorStateObs(guid: string) {
    return this._cacheObs(this.ingestorStateObsStore, guid);
  }
  fleetsObs = new Subject<FleetState[]>();
  fleetStateObsStore: Record<string, Subject<FleetState>> = {};
  getFleetStateObs(name: string) {
    return this._cacheObs(this.fleetStateObsStore, name);
  }
  taskStateObsStore: Record<string, Subject<TaskStateOutput>> = {};
  getTaskStateObs(taskId: string) {
    return this._cacheObs(this.taskStateObsStore, taskId);
  }
  alertRequestsObsStore = new Subject<AlertRequest>();
  alertResponsesObsStore = new Subject<AlertResponse>();
  deliveryAlertObsStore = new Subject<DeliveryAlert>();

  private _cacheObs<T>(store: Record<string, Subject<T>>, key: string): Subject<T> {
    if (!(key in store)) {
      store[key] = new Subject();
    }
    return store[key];
  }
}

export interface TestProviderProps {
  profile?: UserProfile;
  children?: React.ReactNode;
}

/**
 * Provides contexts required for routing and theming.
 */
export const TestProviders = ({ profile = superUser, children }: TestProviderProps) => {
  const authenticator = React.useMemo(() => new StubAuthenticator(), []);
  const resources = React.useMemo<Resources>(
    () => ({
      fleets: {},
      logos: {
        header: '/test-logo.png',
      },
    }),
    [],
  );
  const taskRegistry = React.useMemo<TaskRegistry>(
    () => ({ taskDefinitions: [], pickupZones: [], cartIds: [] }),
    [],
  );
  const settings = React.useMemo<Settings>(
    () => ({ themeMode: 'default', microAppSettings: {} }),
    [],
  );
  const rmfApi = React.useMemo<RmfApi>(() => new MockRmfApi(), []);
  const appController = React.useMemo<AppController>(() => makeMockAppController(), []);

  return (
    <AuthenticatorProvider value={authenticator}>
      <ResourcesProvider value={resources}>
        <TaskRegistryProvider value={taskRegistry}>
          <RmfApiProvider value={rmfApi}>
            <SettingsProvider value={settings}>
              <AppControllerProvider value={appController}>
                <UserProfileProvider value={profile}>{children}</UserProfileProvider>
              </AppControllerProvider>
            </SettingsProvider>
          </RmfApiProvider>
        </TaskRegistryProvider>
      </ResourcesProvider>
    </AuthenticatorProvider>
  );
};

export interface RenderOptions extends Omit<RenderOptions_, 'wrapper'> {
  profile?: UserProfile;
}

/**
 * Helper function to wrap the render function with `TestProviders`.
 */
export function render(ui: React.ReactElement, options?: RenderOptions) {
  const Wrapper: React.FC<React.PropsWithChildren> = ({ children }) => (
    <TestProviders profile={options?.profile}>{children}</TestProviders>
  );
  return render_(ui, { wrapper: Wrapper, ...options });
}
