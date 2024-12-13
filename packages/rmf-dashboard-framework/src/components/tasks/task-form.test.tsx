import { render } from '@testing-library/react';
import { describe, it, vi } from 'vitest';

import { LocalizationProvider } from './../locale';
import { TaskForm } from './task-form';

const mockUser = 'mock_user';
const mockFleets = {
  fleet_1: ['robot_1'],
  fleet_2: ['robot_2', 'robot_3'],
};
const mockCleanZones = ['clean_zone_1', 'clean_zone_2'];
const mockWaypoints = ['waypoint_1', 'waypoint_2', 'waypoint_3'];
const mockPickupZones = ['pickup_zone_1', 'pickup_zone_2'];
const mockCartIds = ['cart_1', 'cart_2', 'cart_3'];
const mockPickupPoints = {
  pickup_1: 'handler_1',
  pickup_2: 'handler_2',
};
const mockDropoffPoints = {
  dropoff_1: 'handler_3',
  dropoff_2: 'handler_4',
};

const onDispatchTask = vi.fn();
const onScheduleTask = vi.fn();
const onEditScheduleTask = vi.fn();
const onSuccess = vi.fn();
const onFail = vi.fn();
const onSuccessFavoriteTask = vi.fn();
const onFailFavoriteTask = vi.fn();
const submitFavoriteTask = vi.fn();
const deleteFavoriteTask = vi.fn();
const onSuccessScheduling = vi.fn();
const onFailScheduling = vi.fn();

describe('Task form', () => {
  it('Task form renders', async () => {
    render(
      <LocalizationProvider>
        <TaskForm
          user={mockUser}
          fleets={mockFleets}
          // taskToDisplay
          cleaningZones={mockCleanZones}
          patrolWaypoints={mockWaypoints}
          pickupZones={mockPickupZones}
          cartIds={mockCartIds}
          pickupPoints={mockPickupPoints}
          dropoffPoints={mockDropoffPoints}
          favoritesTasks={[]}
          // schedule={}
          // taskRequest={}
          onDispatchTask={onDispatchTask}
          onScheduleTask={onScheduleTask}
          onEditScheduleTask={onEditScheduleTask}
          onSuccess={onSuccess}
          onFail={onFail}
          onSuccessFavoriteTask={onSuccessFavoriteTask}
          onFailFavoriteTask={onFailFavoriteTask}
          submitFavoriteTask={submitFavoriteTask}
          deleteFavoriteTask={deleteFavoriteTask}
          onSuccessScheduling={onSuccessScheduling}
          onFailScheduling={onFailScheduling}
          open={true}
        />
      </LocalizationProvider>,
    );
  });
});
