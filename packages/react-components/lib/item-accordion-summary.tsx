import { makeStyles } from '@material-ui/core';
import AccordionSummary from '@material-ui/core/AccordionSummary';
import Typography from '@material-ui/core/Typography';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import React from 'react';
import { joinClasses } from './css-utils';

export interface ItemAccordionSummaryProps {
  title: string;
  status: string;
  classes?: {
    title?: string;
    status?: string;
  };
}

const useStyles = makeStyles((theme) => ({
  content: {
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  hideText: {
    overflow: 'hidden',
    textOverflow: 'ellipsis',
    whiteSpace: 'nowrap',
    maxWidth: '10rem',
  },
  status: {
    borderRadius: theme.shape.borderRadius,
    borderStyle: 'solid',
    border: 2,
    padding: 5,
    width: '4rem',
    textAlign: 'center',
  },
}));

export const ItemAccordionSummary = (props: ItemAccordionSummaryProps): JSX.Element => {
  const classes_ = useStyles();
  const { title, status, classes } = props;

  return (
    <AccordionSummary classes={{ content: classes_.content }} expandIcon={<ExpandMoreIcon />}>
      <Typography variant="h6" className={joinClasses(classes_.hideText, classes?.title)}>
        {title}
      </Typography>
      <Typography className={joinClasses(classes_.status, classes?.status)} variant="button">
        {status}
      </Typography>
    </AccordionSummary>
  );
};

export default ItemAccordionSummary;
