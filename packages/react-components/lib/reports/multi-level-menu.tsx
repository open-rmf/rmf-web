import React, { useState } from 'react';
import { makeStyles } from '@material-ui/core';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import Collapse from '@material-ui/core/Collapse';
import ExpandLess from '@material-ui/icons/ExpandLess';
import ExpandMore from '@material-ui/icons/ExpandMore';

const useStyles = makeStyles((theme) => ({
  textAndIcon: {
    color: theme.palette.text.primary,
  },
}));

interface ListItemBodyProps {
  icon?: JSX.Element;
  title: string;
}

const ListItemBody = (props: ListItemBodyProps): JSX.Element => {
  return (
    <>
      <ListItemIcon>{props.icon}</ListItemIcon>
      <ListItemText primary={props.title} />
    </>
  );
};

interface MenuItemProps {
  icon?: JSX.Element;
  title: string;
  items?: {
    title: string;
    to: string;
  }[];
  onClick?: () => void;
}

const MenuItem = React.memo(
  (props: MenuItemProps): JSX.Element => {
    const classes = useStyles();
    return (
      <ListItem className={classes.textAndIcon} button onClick={props.onClick}>
        <ListItemBody icon={props.icon} title={props.title} />
      </ListItem>
    );
  },
);

export interface ExpandableMultilevelMenuProps {
  icon?: JSX.Element;
  title: string;
  items: MenuItemProps[];
  onClick?: () => void;
}

const ExpandableMenuItem = (props: ExpandableMultilevelMenuProps): JSX.Element => {
  const { items, icon, title } = props;
  const [open, setOpen] = useState(false);
  const classes = useStyles();
  const handleClick = () => {
    setOpen(!open);
  };

  return (
    <div>
      <ListItem className={classes.textAndIcon} button onClick={handleClick}>
        <ListItemBody icon={icon} title={title} />
        {open ? <ExpandLess /> : <ExpandMore />}
      </ListItem>
      <Collapse in={open} timeout="auto" unmountOnExit>
        <MultiLevelMenu menuStructure={items} />
      </Collapse>
    </div>
  );
};

export interface MultilevelMenuProps {
  menuStructure: (ExpandableMultilevelMenuProps | MenuItemProps)[];
}

export const MultiLevelMenu = React.memo(
  (props: MultilevelMenuProps): React.ReactElement => {
    const createList = (items: (ExpandableMultilevelMenuProps | MenuItemProps)[]) => {
      const menu: JSX.Element[] = [];
      items.map((menuItem: ExpandableMultilevelMenuProps | MenuItemProps) => {
        // If it has children's
        if (Array.isArray(menuItem.items) && menuItem.items.length > 0) {
          menu.push(
            <ExpandableMenuItem
              icon={menuItem.icon}
              title={menuItem.title}
              items={menuItem.items}
              key={menuItem.title}
            />,
          );
        } else {
          menu.push(
            <MenuItem
              icon={menuItem.icon}
              title={menuItem.title}
              key={menuItem.title}
              onClick={menuItem.onClick}
            />,
          );
        }
      });
      return menu.concat();
    };

    return <List>{createList(props.menuStructure)}</List>;
  },
);
