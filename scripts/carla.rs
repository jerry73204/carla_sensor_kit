#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! clap = { version = "4", features = ["derive"] }
//! ```

use clap::{Parser, Subcommand};
use std::env;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::Duration;

#[derive(Parser)]
#[command(name = "carla")]
#[command(about = "CARLA server management tool", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Start CARLA server in background
    Start {
        /// Run in headless mode (no display)
        #[arg(long)]
        headless: bool,
        
        /// Set RPC port
        #[arg(short, long, default_value = "3000")]
        port: u16,
        
        /// Set RMW implementation
        #[arg(long)]
        rmw: Option<String>,
    },
    
    /// Stop CARLA server
    Stop,
    
    /// Check CARLA server status
    Status,
    
    /// Restart CARLA server
    Restart {
        /// Run in headless mode (no display)
        #[arg(long)]
        headless: bool,
        
        /// Set RPC port
        #[arg(short, long, default_value = "3000")]
        port: u16,
        
        /// Set RMW implementation
        #[arg(long)]
        rmw: Option<String>,
    },
    
    /// Show server logs
    Logs,
    
    /// Show debug information
    Debug,
}

const CARLA_PATH: &str = "/home/aeon/repos/carla-ros-test/Carla-0.10.0-Linux-Shipping";

fn get_script_dir() -> PathBuf {
    // For rust-script, we need to use a fixed path relative to the project
    // Since this script is always at common/carla.rs
    PathBuf::from("/home/aeon/repos/carla-ros-test/common")
}

fn get_data_dir() -> PathBuf {
    let script_dir = get_script_dir();
    script_dir.join("carla_data")
}

fn ensure_data_dir() -> io::Result<PathBuf> {
    let data_dir = get_data_dir();
    fs::create_dir_all(&data_dir)?;
    Ok(data_dir)
}

fn get_pid_file() -> PathBuf {
    get_data_dir().join("carla_server.pid")
}

fn get_log_file() -> PathBuf {
    get_data_dir().join("carla_server.log")
}

fn read_pid() -> Option<u32> {
    fs::read_to_string(get_pid_file())
        .ok()
        .and_then(|s| s.lines().next().map(|line| line.trim().parse().ok()).flatten())
}

fn read_server_info() -> Option<(u32, u16)> {
    let content = fs::read_to_string(get_pid_file()).ok()?;
    let mut lines = content.lines();
    let pid = lines.next()?.trim().parse().ok()?;
    let port = lines.next()?.trim().parse().ok()?;
    Some((pid, port))
}

fn write_pid(pid: u32) -> io::Result<()> {
    fs::write(get_pid_file(), pid.to_string())
}

fn write_server_info(pid: u32, port: u16) -> io::Result<()> {
    let info = format!("{}\n{}", pid, port);
    fs::write(get_pid_file(), info)
}

fn remove_pid_file() -> io::Result<()> {
    fs::remove_file(get_pid_file())
}

fn is_process_running(pid: u32) -> bool {
    // Check if process exists by sending signal 0
    Command::new("kill")
        .arg("-0")
        .arg(pid.to_string())
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}

fn is_port_open(port: u16) -> bool {
    use std::net::TcpStream;
    
    TcpStream::connect(format!("127.0.0.1:{}", port)).is_ok()
}

fn start_server(headless: bool, port: u16, rmw: Option<String>) -> Result<(), String> {
    // Ensure data directory exists
    ensure_data_dir().map_err(|e| format!("Failed to create data directory: {}", e))?;
    
    // Check if already running
    if let Some(pid) = read_pid() {
        if is_process_running(pid) {
            return Err(format!("CARLA server is already running (PID: {})", pid));
        }
    }
    
    // Check DISPLAY for non-headless mode
    if !headless && env::var("DISPLAY").is_err() {
        return Err(
            "ERROR: DISPLAY environment variable is not set\n\n\
             Please set DISPLAY before running CARLA server:\n\
             export DISPLAY=:0  (or appropriate display number)\n\n\
             Or run in headless mode:\n\
             carla start --headless".to_string()
        );
    }
    
    // Set RMW implementation
    let rmw_impl = rmw.unwrap_or_else(|| {
        env::var("RMW_IMPLEMENTATION").unwrap_or_else(|_| "rmw_fastrtps_cpp".to_string())
    });
    env::set_var("RMW_IMPLEMENTATION", &rmw_impl);
    println!("Using RMW implementation: {}", rmw_impl);
    
    // Source ROS environment
    env::set_var("ROS_DISTRO", "humble");
    
    // Set NVIDIA environment variables
    env::set_var("NVIDIA_VISIBLE_DEVICES", "all");
    env::set_var("NVIDIA_DRIVER_CAPABILITIES", "all");
    
    println!("Starting CARLA server on port {}...", port);
    
    // Build command
    let carla_exe = Path::new(CARLA_PATH).join("CarlaUnreal.sh");
    let log_file_path = get_log_file();
    
    // Build command arguments
    let mut args = vec![
        "-nosound".to_string(),
        format!("-carla-rpc-port={}", port),
        "-quality-level=Low".to_string(),
        "--ros2".to_string(),
    ];
    
    if headless {
        println!("Running in headless/offscreen mode");
        args.push("-RenderOffScreen".to_string());
    } else {
        println!("Running with display (DISPLAY={})", env::var("DISPLAY").unwrap());
    }
    
    // Create log file
    fs::File::create(&log_file_path)
        .map_err(|e| format!("Failed to create log file: {}", e))?;
    
    // Start the process using nohup in background
    let start_script = format!(
        r#"#!/bin/bash
cd {} || exit 1
export RMW_IMPLEMENTATION={}
nohup {} {} >> {} 2>&1 &
"#,
        CARLA_PATH,
        rmw_impl,
        carla_exe.display(),
        args.join(" "),
        log_file_path.display()
    );
    
    // Execute the script
    let output = Command::new("bash")
        .arg("-c")
        .arg(&start_script)
        .output()
        .map_err(|e| format!("Failed to start CARLA: {}", e))?;
    
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let stdout = String::from_utf8_lossy(&output.stdout);
        return Err(format!("Failed to start CARLA:\nstderr: {}\nstdout: {}", stderr, stdout));
    }
    
    // Wait a moment for the process to start
    thread::sleep(Duration::from_secs(2));
    
    // Find the actual CarlaUnreal process PID
    let find_pid_output = Command::new("pgrep")
        .arg("-n")  // Get newest process
        .arg("-f")
        .arg("CarlaUnreal.*-carla-rpc-port=")
        .output()
        .map_err(|e| format!("Failed to find CARLA process: {}", e))?;
    
    let pid_str = String::from_utf8_lossy(&find_pid_output.stdout).trim().to_string();
    if pid_str.is_empty() {
        return Err("Failed to find CARLA process after starting".to_string());
    }
    
    let pid: u32 = pid_str.parse()
        .map_err(|e| format!("Failed to parse PID '{}': {}", pid_str, e))?;
    
    write_server_info(pid, port).map_err(|e| format!("Failed to write server info: {}", e))?;
    
    // Wait a moment to check if process started
    thread::sleep(Duration::from_secs(2));
    
    if is_process_running(pid) {
        println!("CARLA server started successfully (PID: {})", pid);
        println!("Log file: {}", get_log_file().display());
        println!("\nNote: CARLA 0.10.0 may take 30-60 seconds to fully start and open port {}", port);
        println!("Use 'carla status' to check when the server is ready");
        println!("Use 'carla logs' to monitor the startup progress");
        Ok(())
    } else {
        remove_pid_file().ok();
        Err("Failed to start CARLA server".to_string())
    }
}

fn stop_server() -> Result<(), String> {
    let pid = match read_pid() {
        Some(p) => p,
        None => return Err("CARLA server is not running (no PID file)".to_string()),
    };
    
    if !is_process_running(pid) {
        // Try to find any CARLA processes
        let output = Command::new("pgrep")
            .arg("-f")
            .arg("CarlaUnreal")
            .output()
            .ok();
        
        if let Some(output) = output {
            let pids = String::from_utf8_lossy(&output.stdout);
            if !pids.trim().is_empty() {
                println!("Warning: Found CARLA processes not matching PID file. Killing all CARLA processes...");
                Command::new("pkill")
                    .arg("-f")
                    .arg("CarlaUnreal")
                    .output()
                    .ok();
                thread::sleep(Duration::from_secs(2));
            }
        }
        
        remove_pid_file().ok();
        return Err(format!("CARLA server process not found (PID: {})", pid));
    }
    
    println!("Stopping CARLA server (PID: {})...", pid);
    
    // Kill the entire process group
    Command::new("kill")
        .arg("--")
        .arg(format!("-{}", pid))
        .output()
        .ok();
    
    // Also try regular kill
    Command::new("kill")
        .arg(pid.to_string())
        .output()
        .ok();
    
    // Wait for process to terminate
    let mut still_running = false;
    for i in 0..15 {
        thread::sleep(Duration::from_secs(1));
        if !is_process_running(pid) {
            break;
        }
        if i == 5 {
            // Send SIGTERM again after 5 seconds
            println!("Sending SIGTERM again...");
            Command::new("kill")
                .arg(pid.to_string())
                .output()
                .ok();
        }
        if i == 10 {
            still_running = true;
        }
    }
    
    // Force kill if still running
    if still_running && is_process_running(pid) {
        println!("Force killing CARLA server...");
        Command::new("kill")
            .arg("-9")
            .arg(pid.to_string())
            .output()
            .ok();
        
        // Also kill by name as backup
        Command::new("pkill")
            .arg("-9")
            .arg("-f")
            .arg("CarlaUnreal")
            .output()
            .ok();
        
        thread::sleep(Duration::from_secs(1));
    }
    
    // Final check for any remaining CARLA processes
    let check_output = Command::new("pgrep")
        .arg("-f")
        .arg("CarlaUnreal")
        .output()
        .ok();
    
    if let Some(output) = check_output {
        let remaining_pids = String::from_utf8_lossy(&output.stdout);
        if !remaining_pids.trim().is_empty() {
            println!("Warning: Some CARLA processes may still be running: {}", remaining_pids.trim());
            println!("You may need to manually kill them with: pkill -9 -f CarlaUnreal");
        }
    }
    
    remove_pid_file().ok();
    println!("CARLA server stopped");
    Ok(())
}

fn status_server() -> Result<(), String> {
    if let Some(pid) = read_pid() {
        if is_process_running(pid) {
            println!("CARLA server is running (PID: {})", pid);
            println!("Log file: {}", get_log_file().display());
            
            // Check if server is responsive - try to get port from server info
            let port = if let Some((_, stored_port)) = read_server_info() {
                stored_port
            } else {
                env::var("CARLA_RPC_PORT")
                    .unwrap_or_else(|_| "3000".to_string())
                    .parse()
                    .unwrap_or(3000)
            };
            
            if is_port_open(port) {
                println!("Server is listening on port {} ✓", port);
                println!("CARLA server is ready!");
            } else {
                println!("Server process is running but not listening on port {} yet", port);
                println!("This is normal during startup. CARLA 0.10.0 can take 30-60 seconds to initialize.");
                
                // Calculate how long server has been starting
                if let Ok(metadata) = fs::metadata(get_pid_file()) {
                    if let Ok(elapsed) = metadata.modified().unwrap().elapsed() {
                        println!("Server has been starting for {} seconds", elapsed.as_secs());
                    }
                }
            }
        } else {
            println!("CARLA server is not running (stale PID file)");
            remove_pid_file().ok();
        }
    } else {
        println!("CARLA server is not running");
    }
    Ok(())
}

fn show_logs() -> Result<(), String> {
    let log_file = get_log_file();
    if !log_file.exists() {
        return Err("No log file found".to_string());
    }
    
    // Use tail -f
    Command::new("tail")
        .arg("-f")
        .arg(&log_file)
        .status()
        .map_err(|e| format!("Failed to tail log file: {}", e))?;
    
    Ok(())
}

fn debug_info() -> Result<(), String> {
    println!("=== CARLA Debug Information ===");
    println!("CARLA_PATH: {}", CARLA_PATH);
    println!("Script directory: {}", get_script_dir().display());
    println!("Data directory: {}", get_data_dir().display());
    println!("PID file: {}", get_pid_file().display());
    println!("Log file: {}", get_log_file().display());
    println!("RPC_PORT: {}", env::var("CARLA_RPC_PORT").unwrap_or_else(|_| "3000".to_string()));
    println!("DISPLAY: {}", env::var("DISPLAY").unwrap_or_else(|_| "not set".to_string()));
    println!("Current directory: {}", env::current_dir().unwrap().display());
    
    // Check if CARLA directory exists
    let carla_path = Path::new(CARLA_PATH);
    if carla_path.exists() {
        println!("CARLA directory exists");
        
        // Check for required files
        let files = ["CarlaUnreal.sh", "CarlaUnreal/Binaries/Linux/CarlaUnreal-Linux-Shipping"];
        for file in &files {
            let path = carla_path.join(file);
            if path.exists() {
                println!("  ✓ {} exists", file);
            } else {
                println!("  ✗ {} missing", file);
            }
        }
    } else {
        println!("ERROR: CARLA directory not found at {}", CARLA_PATH);
    }
    
    // Check for NVIDIA driver
    if let Ok(output) = Command::new("nvidia-smi")
        .arg("--query-gpu=name,driver_version")
        .arg("--format=csv,noheader")
        .output()
    {
        println!("NVIDIA driver detected");
        print!("{}", String::from_utf8_lossy(&output.stdout));
    } else {
        println!("No NVIDIA driver detected (nvidia-smi not found)");
    }
    
    println!("===============================");
    Ok(())
}

fn main() {
    let cli = Cli::parse();
    
    let result = match cli.command {
        Commands::Start { headless, port, rmw } => start_server(headless, port, rmw),
        Commands::Stop => stop_server(),
        Commands::Status => status_server(),
        Commands::Restart { headless, port, rmw } => {
            stop_server().ok();
            thread::sleep(Duration::from_secs(2));
            start_server(headless, port, rmw)
        },
        Commands::Logs => show_logs(),
        Commands::Debug => debug_info(),
    };
    
    if let Err(e) = result {
        eprintln!("{}", e);
        std::process::exit(1);
    }
}