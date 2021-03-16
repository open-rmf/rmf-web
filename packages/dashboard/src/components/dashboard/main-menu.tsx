import { Divider, List, ListItem, makeStyles, Typography } from '@material-ui/core';
import Debug from 'debug';
import React from 'react';
import DashboardTooltip from 'react-components/lib/tooltip';
import { TooltipsContext } from '../app-contexts';
import { OmniPanelViewIndex } from './dashboard';

const debug = Debug('MainMenu');

const useStyles = makeStyles((theme) => ({
  root: {
    padding: 0,
    backgroundColor: theme.palette.background.paper,
  },
}));

export interface MainMenuProps {
  pushView(view: OmniPanelViewIndex): void;
  setSearch?: () => void;
}

export const MainMenu = React.memo((props: MainMenuProps) => {
  const { showTooltips } = React.useContext(TooltipsContext);
  const { pushView, setSearch } = props;
  debug('render');
  const classes = useStyles();

  const handleMainMenuDoorsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Doors);
    setSearch && setSearch();
  }, [pushView, setSearch]);

  const handleMainMenuLiftsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Lifts);
    setSearch && setSearch();
  }, [pushView, setSearch]);

  const handleMainMenuRobotsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Robots);
    setSearch && setSearch();
  }, [pushView, setSearch]);

  const handleMainMenuDispensersClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Dispensers);
    setSearch && setSearch();
  }, [pushView, setSearch]);

  const handleMainMenuNegotiationsClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Negotiations);
    setSearch && setSearch();
  }, [pushView, setSearch]);

  const handleMainMenuTasksClick = React.useCallback(() => {
    pushView(OmniPanelViewIndex.Tasks);
    setSearch && setSearch();
  }, [pushView, setSearch]);

  return (
    <List className={classes.root} data-component="MainMenu">
      <ListItem data-item="Doors" button={true} onClick={handleMainMenuDoorsClick}>
        <Typography variant="h5">Doors</Typography>
      </ListItem>
      <Divider />

      <ListItem data-item="Lifts" button={true} onClick={handleMainMenuLiftsClick}>
        <Typography variant="h5">Lifts</Typography>
      </ListItem>
      <Divider />

      <ListItem data-item="Robots" button={true} onClick={handleMainMenuRobotsClick}>
        <Typography variant="h5">Robots</Typography>
      </ListItem>
      <Divider />

      <ListItem data-item="Dispensers" button={true} onClick={handleMainMenuDispensersClick}>
        <Typography variant="h5">Dispensers</Typography>
      </ListItem>
      <Divider />

      <ListItem data-item="Negotiations" button={true} onClick={handleMainMenuNegotiationsClick}>
        <DashboardTooltip
          title="This panel shows the negotiations between robots when there are conflicts in trajectories"
          id="negotiations-tooltip"
          enabled={showTooltips}
        >
          <Typography variant="h5">Negotiations</Typography>
        </DashboardTooltip>
      </ListItem>
      <Divider />

      <ListItem data-item="Plans" button={true} onClick={handleMainMenuTasksClick}>
        <Typography variant="h5">Plans</Typography>
      </ListItem>
      <Divider />
    </List>
  );
});

export default MainMenu;
