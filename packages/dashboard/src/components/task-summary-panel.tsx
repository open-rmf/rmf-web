import React from 'react';
import { SpotlightValue } from './spotlight-value';
import Debug from 'debug';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { makeStyles } from '@material-ui/core';
import { TreeView } from '@material-ui/lab';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import { TaskSummaryPanelItem } from './task-summary-panel-item';
const debug = Debug('OmniPanel:TaskSummaryPanel');

export interface TaskSummaryPanelProps {
  allTasks: RomiCore.TaskSummary[];
  spotlight?: Readonly<SpotlightValue<string>>;
  transport?: Readonly<RomiCore.Transport>;
}

export const TaskSummaryPanel = React.memo((props: TaskSummaryPanelProps) => {
  debug('task summary status panel render');

  const { allTasks, spotlight } = props;
  const classes = useStyles();

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    // TODO: spotlight
  }, [spotlight]);

  const [expanded, setExpanded] = React.useState<string[]>([]);
  const [selected, setSelected] = React.useState<string[]>([]);

  const handleToggle = (event: React.ChangeEvent<{}>, nodeIds: string[]) => {
    setExpanded(nodeIds);
  };

  const handleSelect = (event: React.ChangeEvent<{}>, nodeIds: string[]) => {
    setSelected(nodeIds);
  };

  return (
    <TreeView
      className={classes.root}
      onNodeSelect={handleSelect}
      onNodeToggle={handleToggle}
      defaultCollapseIcon={<ExpandMoreIcon />}
      defaultExpanded={['root']}
      defaultExpandIcon={<ChevronRightIcon />}
      expanded={expanded}
      selected={selected}
    >
      {allTasks.map((task) => (
        <TaskSummaryPanelItem task={task} key={task.task_id} />
      ))}
    </TreeView>
  );
});

const useStyles = makeStyles((theme) => ({
  root: {
    padding: '1rem',
  },
}));

export default TaskSummaryPanel;
