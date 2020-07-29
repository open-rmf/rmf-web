import React from 'react';
import { Typography, Divider } from '@material-ui/core';

import { styleTyping } from './BaseComponents/Utils';

const styles: styleTyping = {
    root: {
        margin: '0 auto',
        width: '40%',
    },
    heading: {
        padding: '0.5rem'
    },
    aTag: {
        textDecoration: 'none',
        color: 'rgb(20, 116, 243)'
    },
    example: {
        display: 'flex',
        justifyContent: 'space-between',
        padding: '0.5rem'
    }
}

export default {
    title: 'Fonts',
}

export const Fonts = () => (
    <div style={styles.root}>
        <div style={styles.heading}>
            <Typography variant="body1">
                Our font styles are derived from material-ui's 
                <a style={styles.aTag} href="https://material-ui.com/components/typography/"> Typography</a> tag.
                Click <a href="https://material-ui.com/api/typography/">here</a> to understand more about 
                Typography's api and font options. Below is an example of the <b>body1</b> variant.
            </Typography>
        </div>
        <Divider />
        <div style={styles.example}>
            <Typography variant="body1">Example: </Typography>
            <Typography variant="body1">This is an example of <b>body1</b> font</Typography> 
        </div>
    </div>
)
