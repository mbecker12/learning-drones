import re
import numpy as np
import logging
from scipy.signal import savgol_filter as savgol
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


class DataHandler:
    """
    Handle noisy acceleration data and correct that as needed.
    """
    def __init__(self, time_range):
        self.acceleration = np.zeros(time_range)

