import { makeStyles } from '@material-ui/core';
import AccordionSummary from '@material-ui/core/AccordionSummary';
import Typography from '@material-ui/core/Typography';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import clsx from 'clsx';
import React from 'react';
import { StatusLabel, StatusLabelProps } from './status-label';

export interface ItemAccordionSummaryProps {
  title: string;
  classes?: {
    title?: string;
  };
  statusProps?: StatusLabelProps;
}

const useStyles = makeStyles(() => ({
  content: {
    alignItems: 'center',
    justifyContent: 'space-between',
    overflow: 'hidden',
  },
  hideText: {
    overflow: 'hidden',
    textOverflow: 'ellipsis',
    whiteSpace: 'nowrap',
  },
}));

export const ItemAccordionSummary = (props: ItemAccordionSummaryProps): JSX.Element => {
  const classes_ = useStyles();
  const { title, classes, statusProps } = props;

  return (
    <AccordionSummary classes={{ content: classes_.content }} expandIcon={<ExpandMoreIcon />}>
      <Typography variant="h6" className={clsx(classes_.hideText, classes?.title)}>
        {title}
      </Typography>
      <StatusLabel {...statusProps} />
    </AccordionSummary>
  );
};

export default ItemAccordionSummary;
