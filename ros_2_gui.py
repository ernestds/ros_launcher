import os
import subprocess

import xml.etree.ElementTree as ET
import ament_index_python.packages
from textual.app import App
import textual
from textual.widget import Widget
from textual.widgets import Tree,    Label,    Button
import glob
from textual.app import App, ComposeResult
from textual.containers import VerticalScroll
from textual.widgets import Input, Static
import textual.widgets
from textual.app import App, ComposeResult,Binding
from textual.containers import Container, Horizontal,Vertical, VerticalScroll
from textual.widgets import Header, Static
from textual.widgets import TextLog
from textual import Logger
import traceback
import asyncio
import select
import tempfile
import time
from pathlib import Path
debug_size = 1000
class Debugger(Container):
    values = None
    def __init__(self, *args, **kwargs):
        print(args)
        super().__init__(*args, **kwargs)
        self.values = []
    def compose(self) -> ComposeResult:
        with VerticalScroll(id="left-pane"):
            yield Static('',id='text')
    def debug(self, value):
        self.values.append(value)
        if len(self.values) > debug_size:
            self.values.pop(0)
        # self.query_one("#left-pane").update()
        
        self.query_one("#text").update("[bold magenta]World[/bold magenta]" + "\n".join(self.values))
class ROSLaunchGUI(App):
    tree = None
    textlog = None
    CSS_PATH = "ros_gui.css"
    BINDINGS = [Binding("enter", "enter_password('enter')", "",priority=True),Binding("escape", "enter_password('esc')", "")]
    sudo_prompt = False
    current_selected_data = None
    not_executed = True
    async def read_stream(self,stream,stream_name):
        i=0
        while True:
            try:
                # self.textlog.write("asfafasf")
                line = await asyncio.wait_for(stream.readline(), timeout=2)
                self.textlog.write(f"Line: {line}")
                if line:
                    self.textlog.write(line)
                    return True
                if i > 10:
                    return False
                i+=1
                # else: 
                    # return False
            except asyncio.TimeoutError:
                return False
                # self.textlog.write("timeout")
                # break
    def close_sudo_prompt(self):
        self.query_one("#password").remove()
        self.query_one("#hor1").mount(Button("Launch",id="launch_button"))
        self.query_one("#hor1").mount(Button("Reload Packages",id="reload_button"))
        self.query_one("#hor1").mount(Input(placeholder="Custom preamble: custom_preamble (source ros or sth else for example) + r'echo -en \"\033]0\;' + name + r'\a\" && source install/setup.bash && '",id="preamble"))
        self.query_one("#hor2").mount(Button("Build",id="build_button"))
        self.query_one("#hor2").mount(Button("Source install/devel.setup",id="source_button"))
        self.query_one("#hor2").mount(Input(placeholder="Arguments that go after build/launch/run (colcon build args etc)",id="arguments"))
        
    def open_sudo_prompt(self):
        self.query_one("#hor1").mount(Input(placeholder="Sudo password",id="password",password=True))
        self.query_one("#launch_button").remove()
        self.query_one("#reload_button").remove()
        self.query_one("#preamble").remove()
        self.query_one("#build_button").remove()
        self.query_one("#source_button").remove()
        self.query_one("#arguments").remove()
        
    async def action_enter_password(self,key:str) -> None:
        
        if self.sudo_prompt:
            if key == "esc":
                self.sudo_prompt = False
                self.textlog.write("Cancelled")
                self.close_sudo_prompt()
            if key == "enter":
                if self.not_executed:
                    self.not_executed = False
                    self.textlog.write("Enter")
                    self.process = await asyncio.create_subprocess_exec(*self.subprocess_command,stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                    self.process.stdin.write(self.query_one("#password").value.encode()+b"\n")
                    # self.textlog.write("1")
                    try:                    
                        await self.process.stdin.drain()
                        success = await self.read_stream(self.process.stderr,"aa")
                        self.textlog.write(str(success))
                        if success:
                            self.textlog.write("Success")
                            self.close_sudo_prompt()
                            self.sudo_prompt = False
                            self.process.stdin.close()
                        else:
                            self.textlog.write(str(self.process.pid))
                            await asyncio.sleep(1)
                            try:
                                self.process.kill()
                            except:
                                self.textlog.write(traceback.format_exc())
                            # subprocess.check_call(["sudo", "kill", str(self.process.pid)])
                            # os.waitpid(self.process.pid, 0)
                            self.textlog.write("Wrong password")
                    except ConnectionResetError:
                        self.close_sudo_prompt()
                        self.textlog.write("Fail")
                        self.refresh()
                        self.sudo_prompt = False

                    await asyncio.sleep(0.5)
                    self.not_executed = True
                
                
                
    def get_package_list_in_current_workspace(self):
        packages = ament_index_python.packages.get_packages_with_prefixes()
        current_workspace = Path.cwd()
        print(packages.items())
        return {pkg: path for pkg, path in packages.items() if str(current_workspace) in str(Path(path))}
       
    def populate_tree(self):
        self.tree.clear()
        
        
        package_list = self.get_package_list_in_current_workspace()
        for pkg, pkg_path in package_list.items():
            pkg_path = ament_index_python.packages.get_package_share_directory(pkg)
            parent = self.tree.root.add(pkg, expand=True)
            # share_folder = pkg_path
            if os.path.exists(pkg_path):
                parent_launch = parent.add("launch", expand=True)
                
                launch_dir = os.path.join(pkg_path, "launch")
                for file in sorted(glob.glob(pkg_path + '/**/*launch.py', recursive=True)):
                    # self.tree.insert(parent, "end", text=os.path.basename(file), values=('launch',pkg,os.path.basename(file),))
                    parent_launch.add_leaf(os.path.basename(file), data={"type":"launch","filepath":os.path.join(launch_dir, file),"pkg":pkg})
            exec_dir = os.path.join(pkg_path, "../../lib", pkg)
            if os.path.exists(exec_dir):
                # executable_node = self.tree.insert(parent, "end", text="executables")
                parent_exec = parent.add("executable", expand=True)
                for file in sorted(os.listdir(exec_dir)):
                    if os.access(os.path.join(exec_dir, file), os.X_OK):
                        parent_exec.add_leaf(file, data={"type":"run","filepath":os.path.join(launch_dir, file),"pkg":pkg})
        # for pkg in sorted(rp.list()):
        #     pkg_path = rp.get_path(pkg)
        #     if pkg_path.startswith(workspace_prefix):
        #         parent = self.tree.root.add(pkg, expand=True)
                
        #         launch_dir = os.path.join(pkg_path, "launch")
        #         if os.path.exists(launch_dir):
        #             for file in sorted(os.listdir(launch_dir)):
        #                 if file.endswith(".launch"):
        #                     parent.add_leaf(file, data={"type":"launch","filepath":os.path.join(launch_dir, file),"pkg":pkg})
        #                     # self.textlog.write(file)
    async def on_button_pressed(self, event):
        button_id = event.button.id
        if button_id == "reload_button":
            self.populate_tree()
        elif button_id == "launch_button":
            if self.current_selected_data is not None:
                await self.launch(self.current_selected_data["filepath"])	
        elif button_id == "build_button":
            self.textlog.write("buidl: ")
            await self.build()
            self.textlog.write("buidl: ")
        elif button_id == "source_button":
            await self.source_folder()
    
    async def source_folder(self):
        # commands = ["env", "-i","bash","-c", 'source install/setup.bash && env']
        # process = subprocess.Popen(commands, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # stdout, stderr = process.communicate()
        # for line in stdout.decode('utf-8').strip('\n').strip().split('\n'):
        #     split = line.split('=')
        #     left = split[0].strip()
        #     right = "=".join(split[1:])
        #     os.environ[left] = right
        #     os.environ["aaaaaa"] = "aaaaaa"
        # self.build()
        # self.textlog.write("Doing: ")
        commands = ["env", "-i", "bash", "-c", 'source install/setup.bash && env']
        process = await asyncio.create_subprocess_exec(*commands, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        stdout, stderr = await process.communicate()
        dic = {}
        for line in stdout.decode('utf-8').strip('\n').strip().split('\n'):
            split = line.split('=')
            left = split[0].strip()
            right = "=".join(split[1:])
            dic[left] = right
            
            if left in os.environ:
                self.textlog.write(f"{left} = {os.environ[left]}\n {right}")
            # os.environ[left] = right
        os.environ["AMENT_PREFIX_PATH"] = dic["AMENT_PREFIX_PATH"]
        # os.environ["PYTHONPATH"] = dic["PYTHONPATH"]
        # os.environ["PATH"] = dic["PATH"]
        # os.environ["LD_LIBRARY_PATH"] = dic["LD_LIBRARY_PATH"]
        # os.environ["_"] = dic["_"]
            # self.textlog.write("AAA")
        # process.terminate()
        # await self.build()
        # self.textlog.write("Doing: ")
    async def build(self):
        arguments = self.query_one("#arguments").value
        if arguments is None or arguments == "":
            arguments = ""
        command = "colcon build " + arguments
        self.textlog.write(command)
        process = await asyncio.create_subprocess_exec('bash', '-c', f"wt.exe -w 0 new-tab -p $WT_PROFILE_ID -d . bash -ci '{command} && bash'", stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        stdout, stderr = await process.communicate()
    # def build(self):
    #     command = "colcon build"
    #     process = subprocess.Popen(['bash','-c', f"wt.exe -w 0 new-tab -p $WT_PROFILE_ID -d . bash -ci '{command} && bash'"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    async def launch(self,filepath):
        # workspace_setup_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "devel", "setup.bash")
        # command = f'source {workspace_setup_script}; roslaunch {filepath}; exec bash'
        # self.textlog.write("Doing: " + command)
        # subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command])
        arguments = self.query_one("#arguments").value
        if arguments is None or arguments == "":
            arguments = ""
        custom_preamble = self.query_one("#preamble").value
        if custom_preamble is None or custom_preamble == "":
            custom_preamble = ""
        else:
            custom_preamble = custom_preamble + " && "
        file_type = self.current_selected_data["type"]
        pkg = self.current_selected_data["pkg"]
        name = os.path.basename(self.current_selected_data["filepath"])
        
        # ${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u\[\033[00m\]:\ [\033[01;34m\]\W\[\033[00m\]\$ 
        # + r'export A=\"${debian_chroot:+($debian_chroot)}\[\033[01\;32m\]\u@\h\[\033[00m\]:\[\033[01\;34m\]\w\[\033[00m\]\$ \" && ' + r'echo -en \"\033]0\;' + name + r'\a\" 
        preamble = custom_preamble + r'source install/setup.bash && '
        # preamble = custom_preamble + r'echo -en \"\033]0\;' + name + r'\a\" && source install/setup.bash && '
        # preamble
        if file_type == 'launch':
            command = preamble + f'(ros2 launch {pkg} {name} {arguments} || true) '
        if file_type == 'run':
            command = preamble + f'(ros2 run {pkg} {name} {arguments} || true) '
        
        if "WSLENV" in os.environ:
            self.subprocess_command = ['bash','-c',f"wt.exe -w 0 new-tab -p $WT_PROFILE_ID -d . bash -ci '{command} && bash'"]
        else:
            self.subprocess_command = ['gnome-terminal', '--', 'bash', '-ci', "'" + command + " && bash'"]
        process = subprocess.Popen(self.subprocess_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,stdin=subprocess.PIPE)
        stderr, stdout = process.communicate()
        outmsg = stdout.decode()
        errmsg = stderr.decode()
        self.textlog.write("stdout: " + outmsg)
        self.textlog.write("stderr: " + errmsg)
        
        # self.subprocess_command = ['sudo','-S','gnome-terminal', '--', 'bash', '-c', command]
        # self.textlog.write("Doing: " + str(self.subprocess_command))
        # process = subprocess.Popen(self.subprocess_command,
        #                  stdin=subprocess.PIPE,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # stderr, stdout = process.communicate()
        # outmsg = stdout.decode()
        # if 'password' in outmsg:
        #     self.sudo_prompt = True
        #     self.textlog.write("Sudo password required (press esc to cancel)")
        #     self.open_sudo_prompt()
        if errmsg != "" or outmsg != "":
            
            self.subprocess_command = ['sudo','-S'] + self.subprocess_command
            # self.sudo_prompt = True
            self.textlog.write("this doesnt work Error? Try sudo (press esc to cancel) + " + str(self.subprocess_command))
            # self.open_sudo_prompt()   
            
            

        # a = r'export PS1=\"aaa\" && echo $PS1 ksasak'
        # process = subprocess.Popen(['bash','-c', f"wt.exe -w 0 new-tab -p $WT_PROFILE_ID -d . bash -ci '{a} && bash'"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
        # process = subprocess.Popen(subprocess_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    async def on_tree_node_highlighted(self, event):
        # self.populate_tree()
        if self.sudo_prompt:
            self.sudo_prompt = False
            self.close_sudo_prompt()
        if not event.node.data:
            self.current_selected_data = None
            return
        
        name_node = self.query_one("#name")
        # self.textlog.write(str(event.node))
        description_node = self.query_one("#description")
        filepath = event.node.data["filepath"]
        self.current_selected_data = event.node.data

        display_name = filepath
        
        description = ""
        # parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        # root = ET.parse(filepath, parser)
        # for comment in root.iter():
        #     if "function Comment" in str(comment.tag):
        #         text = comment.text.strip()
        #         if text.startswith("name:"):
        #             display_name = text[5:].strip()
        #         elif text.startswith("description:"):
        #             description = text[12:].strip()
        name_node.update("Name: " + display_name)
        description_node.update("Description: " + "to do: description")
        

    def compose(self) -> ComposeResult:
        yield Header()
        with Vertical():
            
            # with Container(id="app-fsd"):
            #     with VerticalScroll(id="left-pane"):
            self.textlog = TextLog(highlight=True, markup=True,auto_scroll=True,id="textlog")
            yield self.textlog
            # yield Debugger(id='debugger')
            with Container(id="app-grid"):
                with VerticalScroll(id="left-pane"):
                    self.tree: Tree[dict] = Tree("ROS")
                    self.populate_tree()
                    yield self.tree
            yield Static('Name: ',id='name')
            yield Static('Description: ',id='description')
            
            with Container(id = 'buttons_container'):
                with Horizontal(classes = "button_line",id="hor1"):
                    yield Button("Launch",id="launch_button")
                    yield Button("Reload Packages",id="reload_button")
                    yield Input(placeholder="Custom preamble: custom_preamble (source ros or sth else for example) + r'echo -en \"\033]0\;' + name + r'\a\" && source install/setup.bash && '",id="preamble")
                with Horizontal(classes = "button_line",id="hor2"):
                    yield Button("Build",id="build_button")
                    yield Button("Source install/devel.setup",id="source_button")
                    yield Input(placeholder="Arguments that go after build/launch/run (colcon build args etc)",id="arguments")
    async def refresh_tree(self):
        self.tree.clear()
        await self.tree.add_data(self.packages)



# print(" ".join(commands))
# print("done")
if __name__ == "__main__":
    app = ROSLaunchGUI()
    app.run()
    