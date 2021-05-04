import React from 'react';
import { createStyles, makeStyles } from '@material-ui/core';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import Badge from '@material-ui/core/Badge';

const useStyles = makeStyles(() =>
  createStyles({
    logo: {
      maxWidth: 120,
      opacity: 1,
    },
  }),
);

interface BannerTabProps {
  logoPath?: string;
}

export const BannerTab = (props: BannerTabProps): JSX.Element => {
  const { logoPath } = props;
  const classes = useStyles();

  return (
    <Tabs
      variant="standard"
      value={0}
      TabIndicatorProps={{
        style: {
          display: 'none',
        },
      }}
    >
      {logoPath && (
        <Tab
          label={
            <Badge badgeContent={<img src={logoPath} alt="logo" className={classes.logo} />} />
          }
        />
      )}
    </Tabs>
  );
};
