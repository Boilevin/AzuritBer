/* This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)

*/

#ifndef SCREEN_H
#define SCREEN_H

#include <Arduino.h>

class Screen
{
  public:
    void init();
    void refreshMowScreen();
    void refreshStationScreen();
    void refreshWaitScreen() ;
    void refreshErrorScreen() ;
    void refreshTrackScreen() ;
   
  
    
};





#endif
