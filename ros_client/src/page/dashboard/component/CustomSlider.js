import {withStyles} from '@material-ui/core/styles';
import {Slider} from '@material-ui/core';

export const CustomSlider = withStyles({
    root: {
        color: '#017a8f',
        height: 10,
    },
    thumb: {
        height: 20,
        width: 20,
        backgroundColor: '#ffffff',
        border: '2px solid currentColor',
        marginTop: -8,
        marginLeft: -12,
        '&:focus, &:hover, &$active': {
            boxShadow: 'inherit',
        },
    },
    active: {},
    rail: {
        height: 5,
        borderRadius: 10,
    },
})(Slider);