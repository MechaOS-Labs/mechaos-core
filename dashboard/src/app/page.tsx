import { Button, buttonVariants } from "@/components/ui/button";
import { Rocket, Star } from "lucide-react";
import Link from "next/link";

export default function Home() {
  return (
    <div className="font-sans grid grid-rows-[20px_1fr_20px] items-center justify-items-center min-h-screen p-8 pb-20 gap-16 sm:p-20  ">
      <main className="flex flex-col gap-[32px] row-start-2 items-center text-center">
        {/* Logo placeholder - replace with MechaOS logo */}
        <div className="flex items-center gap-3">
          <h1 className="text-4xl font-bold -tracking-normal">MechaOS</h1>
        </div>

        <div className="max-w-2xl">
          <h2 className="text-2xl font-semibold  mb-4">
            Decentralized Robot Control Platform
          </h2>
          <p className="text-lg mb-8">
            The future of robotics meets blockchain technology. Control
            real-world robots through Ethereum smart contracts.
          </p>
        </div>

        <div className="flex gap-4 items-center flex-col sm:flex-row">
          <Button className="">
            <Rocket size={24} />

            <h2>Coming Soon</h2>
          </Button>
          <Link
            className={buttonVariants({
              variant: "outline",
            })}
            href="https://github.com/MechaOS-Labs/mechaos-core"
            target="_blank"
            rel="noopener noreferrer"
          >
            <span className="mr-2">
              <Star className="text-yellow-300" fill="#f2ff00" />
            </span>
            Star on GitHub
          </Link>
        </div>
      </main>
    </div>
  );
}
