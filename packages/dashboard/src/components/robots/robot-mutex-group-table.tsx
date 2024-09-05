import { TableContainer, Typography } from '@mui/material';
import React from 'react';
import { ConfirmationDialog, MutexGroupData, MutexGroupTable } from 'react-components';

import { useAppController } from '../../hooks/use-app-controller';
import { useRmfApi } from '../../hooks/use-rmf-api';

const RefreshMutexGroupTableInterval = 5000;

export const RobotMutexGroupsTable = () => {
  const rmfApi = useRmfApi();
  const appController = useAppController();

  const [mutexGroups, setMutexGroups] = React.useState<Record<string, MutexGroupData>>({});
  const [selectedMutexGroup, setSelectedMutexGroup] = React.useState<MutexGroupData | null>(null);

  const robotIdentifierDelimiter = '/';

  const generateRobotIdentifier = (fleet: string, robot: string) => {
    return `${fleet}${robotIdentifierDelimiter}${robot}`;
  };

  const getFleetFromRobotIdentifier = (robotIdentifier: string) => {
    const split = robotIdentifier.split(robotIdentifierDelimiter);
    if (split.length !== 2) {
      console.error(`Unable to parse fleet from robot identifier: ${robotIdentifier}`);
      return null;
    }
    return split[0];
  };

  const getRobotFromRobotIdentifier = (robotIdentifier: string) => {
    const split = robotIdentifier.split(robotIdentifierDelimiter);
    if (split.length !== 2) {
      console.error(`Unable to parse robot from robot identifier: ${robotIdentifier}`);
      return null;
    }
    return split[1];
  };

  React.useEffect(() => {
    const refreshMutexGroupTable = async () => {
      const fleets = (await rmfApi.fleetsApi.getFleetsFleetsGet()).data;
      const updatedMutexGroups: Record<string, MutexGroupData> = {};
      for (const fleet of fleets) {
        if (!fleet.name || !fleet.robots) {
          continue;
        }

        for (const robot of Object.values(fleet.robots)) {
          if (!robot.mutex_groups || !robot.name) {
            continue;
          }
          const robotIdentifier = generateRobotIdentifier(fleet.name, robot.name);

          if (robot.mutex_groups.locked) {
            for (const locked of robot.mutex_groups.locked) {
              if (updatedMutexGroups[locked]) {
                updatedMutexGroups[locked].lockedBy = robotIdentifier;
              } else {
                updatedMutexGroups[locked] = {
                  name: locked,
                  lockedBy: robotIdentifier,
                  requestedBy: [],
                };
              }
            }
          }
          if (robot.mutex_groups.requesting) {
            for (const requesting of robot.mutex_groups.requesting) {
              if (updatedMutexGroups[requesting]) {
                updatedMutexGroups[requesting].requestedBy.push(robotIdentifier);
              } else {
                updatedMutexGroups[requesting] = {
                  name: requesting,
                  lockedBy: undefined,
                  requestedBy: [robotIdentifier],
                };
              }
            }
          }
        }
      }

      // Filter intermediate mutex groups which are not locked, but just
      // requested by robots
      for (const mutexGroupName of Object.keys(updatedMutexGroups)) {
        if (!updatedMutexGroups[mutexGroupName].lockedBy) {
          delete updatedMutexGroups[mutexGroupName];
        }
      }

      setMutexGroups(updatedMutexGroups);
    };

    // Initialize table
    (async () => {
      await refreshMutexGroupTable();
    })();

    // Set up regular interval to refresh table
    const refreshInterval = window.setInterval(
      refreshMutexGroupTable,
      RefreshMutexGroupTableInterval,
    );
    return () => {
      clearInterval(refreshInterval);
    };
  }, [rmfApi]);

  const handleUnlockMutexGroup = React.useCallback<React.MouseEventHandler>(async () => {
    if (!selectedMutexGroup || !selectedMutexGroup.lockedBy) {
      return;
    }
    const fleet = getFleetFromRobotIdentifier(selectedMutexGroup.lockedBy);
    const robot = getRobotFromRobotIdentifier(selectedMutexGroup.lockedBy);
    if (!fleet || !robot) {
      return;
    }

    try {
      await rmfApi.fleetsApi?.unlockMutexGroupFleetsNameUnlockMutexGroupPost(
        fleet,
        robot,
        selectedMutexGroup.name,
      );
      appController.showAlert(
        'success',
        `Requested to unlock mutex group ${selectedMutexGroup.name} for ${fleet}:${robot}`,
      );
    } catch (e) {
      appController.showAlert(
        'error',
        `Failed to unlock mutex group ${selectedMutexGroup.name} for ${fleet}:${robot}: ${
          (e as Error).message
        }`,
      );
    }
    setSelectedMutexGroup(null);
  }, [selectedMutexGroup, rmfApi, appController]);

  return (
    <TableContainer sx={{ height: '100%' }}>
      <MutexGroupTable
        mutexGroups={Object.values(mutexGroups)}
        onMutexGroupClick={(_ev, mutexGroup) => {
          setSelectedMutexGroup(mutexGroup);
        }}
      />
      <ConfirmationDialog
        confirmText="Confirm unlock"
        cancelText="Cancel"
        open={selectedMutexGroup !== null && selectedMutexGroup.lockedBy !== undefined}
        title={'Mutex group manual unlock'}
        submitting={undefined}
        onClose={() => setSelectedMutexGroup(null)}
        onSubmit={handleUnlockMutexGroup}
      >
        {selectedMutexGroup && selectedMutexGroup.lockedBy ? (
          <Typography>
            Confirm unlock mutex group [{selectedMutexGroup.name}] for [
            {selectedMutexGroup.lockedBy}]?
          </Typography>
        ) : (
          <Typography>Confirm unlock mutex group?</Typography>
        )}
      </ConfirmationDialog>
    </TableContainer>
  );
};

export default RobotMutexGroupsTable;
